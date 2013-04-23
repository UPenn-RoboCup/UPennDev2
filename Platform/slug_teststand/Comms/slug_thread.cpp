#include <unistd.h>
#include <string.h>
#include "dcm.h"
#include "config.h"
#include "slug_config.h"
#include "slug_thread.h"

// slug_thread : motor slug communication thread
// author : Mike Hopkins
///////////////////////////////////////////////////////////////////////////

#define SDO_TIMEOUT  0.10
#define SYNC_TIMEOUT 0.01

extern Dcm dcm;
extern Config config;

slug_thread::slug_thread()
{
  m_slug_ids.resize(0);
  m_node_ids.resize(0);
  m_joint_ids.resize(0);
  sprintf(m_can_interface, "\0");
}

void slug_thread::set_slug_ids(int *ids, int n_ids)
{ 
  // initialize slug, node and joint ids
  m_slug_ids = std::vector<int>(ids, ids + n_ids);
  m_node_ids.resize(n_ids);
  m_joint_ids.resize(n_ids);
  for (int i = 0; i < n_ids; i++) 
  {
    m_node_ids[i] = slug_config_node_ids[ids[i]];
    m_joint_ids[i] = slug_config_joint_ids[ids[i]];
  }
}

void slug_thread::set_can_interface(const char *interface)
{
  // initialize can interface
  strncpy(m_can_interface, interface, 128);
}

void slug_thread::emcy_callback(int node_id, void *user_data)
{
  // terminate comms manager upon receiving an emergency message
  int error_code = 0;
  slug_thread *this_slug_thread = (slug_thread *)user_data;
  for (int i = 0; i < this_slug_thread->m_slug_ids.size(); i++)
  {
    if (node_id == this_slug_thread->m_slug[i]->get_node_id())
      error_code = this_slug_thread->m_slug[i]->get_error_code();
  }
  sprintf(this_slug_thread->m_error_message,
    "node %d: emcy error %x", node_id, error_code
  );
  this_slug_thread->m_stop_request = 1;
}

void slug_thread::initialize_controllers()
{
  // get joint position and motor force biases
  std::vector<double> joint_position_bias = 
    config.get_double_vector("bias.joint_position_bias");
  std::vector<double> motor_force_bias =
    config.get_double_vector("bias.motor_force_bias");

  // initialize motor slugs
  for (int i = 0; i < m_slug_ids.size(); i++)
  {
    int slug_id = m_slug_ids[i];
    int node_id = m_node_ids[i];

    // enable preoperational state
    m_master.enter_preoperational_state(node_id);
    usleep(1e3);

    // configure object dictionary
    fprintf(stderr, "[ node %d ] configuring pdo settings\n", node_id);
    assert(m_master.configure_pdo_settings(node_id));

    fprintf(stderr, "[ node %d ] configuring sdo settings\n", node_id);
    assert(m_master.configure_sdo_settings(
      node_id, slug_config_default_settings
    ));
    assert(m_master.configure_sdo_settings(
      node_id, slug_config_custom_settings[slug_id]
    ));

    // configure joint position and motor force sensor bias
    for (int j = 0; j < 2; j++)
    {
      int joint_id = m_joint_ids[i][j];
      if (joint_id > -1) 
      {
        m_slug[i]->set_joint_position_sensor_bias(
          joint_position_bias[joint_id], j + 1
        );
        m_slug[i]->set_motor_force_sensor_bias(
          motor_force_bias[joint_id], j + 1
        );
        assert(m_master.sdo_download(
          node_id, SLUG_JOINT_POSITION_SENSOR_BIAS, j + 1, SDO_TIMEOUT
        )); 
        assert(m_master.sdo_download(
          node_id, SLUG_MOTOR_FORCE_SENSOR_BIAS, j + 1, SDO_TIMEOUT
        )); 
      }
    }

    // enable remote node
    usleep(1e3);
    m_master.start_remote_node(node_id);
  }
}

void slug_thread::update_controller_inputs()
{
  // update controller inputs
  for (int i = 0; i < m_slug_ids.size(); i++)
  {
    int slug_id = m_slug_ids[i];
    int node_id = m_node_ids[i];
    int enabled = 1;
    for (int j = 0; j < 2; j++)
    {
      int joint_id = m_joint_ids[i][j];
      if (joint_id > -1) 
      {
        // update joint enable 
        enabled &= (int)dcm.joint_enable[joint_id];

        // update object dictionary
        m_slug[i]->set_value(
          SLUG_JOINT_FORCE_SETPOINT, j + 1, dcm.joint_force[joint_id]
        );
        m_slug[i]->set_value(
          SLUG_JOINT_POSITION_SETPOINT, j + 1, dcm.joint_position[joint_id]
        );
        m_slug[i]->set_value(
          SLUG_JOINT_VELOCITY_SETPOINT, j + 1, dcm.joint_velocity[joint_id]
        );
        m_slug[i]->set_value(
          SLUG_JOINT_POSITION_P_GAIN, j + 1, dcm.joint_p_gain[joint_id]
        );
        m_slug[i]->set_value(
          SLUG_JOINT_POSITION_I_GAIN, j + 1, dcm.joint_i_gain[joint_id]
        );
        m_slug[i]->set_value(
          SLUG_JOINT_POSITION_D_GAIN, j + 1, dcm.joint_d_gain[joint_id]
        );

        // send rpdo 1/2 to update setpoints 
        m_master.send_rpdo(node_id, j + 1);
        
        // send rpdo 3/4 to update pid gains
        if ((dcm.joint_p_gain_updated[joint_id] == 1)
          ||(dcm.joint_i_gain_updated[joint_id] == 1)
          ||(dcm.joint_d_gain_updated[joint_id] == 1))
        {
          dcm.joint_p_gain_updated[joint_id] = 0; 
          dcm.joint_i_gain_updated[joint_id] = 0; 
          dcm.joint_d_gain_updated[joint_id] = 0; 
          m_master.send_rpdo(node_id, j + 3);
        }
      }
    }

    // update operating mode
    int operating_mode =
      enabled ? SLUG_OP_MODE_JOINT_IMPEDANCE : SLUG_OP_MODE_IDLE;
    int operating_mode_display =
      m_slug[i]->get_value(SLUG_OPERATING_MODE_DISPLAY, 0x00);
    if (operating_mode != operating_mode_display)
    {
      m_slug[i]->set_value(SLUG_OPERATING_MODE, 0x00, operating_mode);
      m_master.sdo_download(node_id, SLUG_OPERATING_MODE, 0x00, 0.01);
    }
  }
}

void slug_thread::update_controller_outputs()
{
  // get transmit can_ids for sync polling
  co_can_id sync_ids[256];
  for (int i = 0; i < m_node_ids.size(); i++)
  {
    int node_id = m_node_ids[i];
    sync_ids[2*i] = CO_TPDO1 | node_id;
    sync_ids[2*i+1] = CO_TPDO2 | node_id;
  }

  // receive sync data 
  m_master.receive(sync_ids, 2*m_node_ids.size(), SYNC_TIMEOUT);
  m_master.flush_can_channel();

   // update controller outputs 
  for (int i = 0; i < m_slug_ids.size(); i++)
  {
    int slug_id = m_slug_ids[i];
    int node_id = m_node_ids[i];
    for (int j = 0; j < 2; j++)
    {
      int joint_id = m_joint_ids[i][j];
      if (joint_id > -1) 
      {
        // update DCM
        dcm.joint_force_sensor[joint_id] = m_slug[i]->get_value(
          SLUG_JOINT_FORCE_ESTIMATE, j + 1
        );
        dcm.joint_position_sensor[joint_id] = m_slug[i]->get_value(
          SLUG_JOINT_POSITION_ESTIMATE, j + 1
        );
        dcm.joint_velocity_sensor[joint_id] = m_slug[i]->get_value(
          SLUG_JOINT_VELOCITY_ESTIMATE, j + 1
        );
        dcm.motor_force_sensor[joint_id] = m_slug[i]->get_value( 
          SLUG_MOTOR_FORCE_ESTIMATE, j + 1
        );
      }
    }
  }
}

void slug_thread::entry()
{
  // initialize slug_slave objects
  for (int i = 0; i < m_slug_ids.size(); i++)
  {
    int slug_id = m_slug_ids[i];
    int node_id = m_node_ids[i];
    m_slug[i] = new slug_slave();
    m_slug[i]->set_node_id(node_id);
    m_slug[i]->register_sdo_settings(slug_config_default_settings);
    m_slug[i]->register_sdo_settings(slug_config_custom_settings[slug_id]);
    m_slug[i]->register_pdo_parameters(slug_config_pdo_parameters);
    m_slug[i]->register_pdo_mappings(slug_config_pdo_mappings);
  }

  // initialize CAN channel
  assert(0 == m_channel.open_channel(m_can_interface));

  // initialize CANopen master
  m_master.set_can_channel(&m_channel);
  m_master.enable_debug_print(true);
  m_master.set_default_timeout(SDO_TIMEOUT);
  for (int i = 0; i < m_slug_ids.size(); i++)
  {
    m_master.register_slave(m_slug[i]);
  }

  // initial motor slugs
  initialize_controllers();

  // enable emergency message handler
  m_master.register_emcy_callback(&slug_thread::emcy_callback, this);

  // initialize DCM shared memory
  m_master.send_sync();
  update_controller_outputs();
  for (int i = 0; i < m_slug_ids.size(); i++)
  {
    for (int j = 0; j < 2; j++)
    {
      int joint_id = m_joint_ids[i][j];
      if (joint_id > -1) 
      {
        dcm.joint_enable[joint_id] = 1;
        dcm.joint_p_gain[joint_id] = 0;
        dcm.joint_i_gain[joint_id] = 0;
        dcm.joint_d_gain[joint_id] = 0;
        dcm.joint_force[joint_id] = 0;
        dcm.joint_position[joint_id] = dcm.joint_position_sensor[joint_id];
        dcm.joint_velocity[joint_id] = 0; 
      }
    }
  }
}

void slug_thread::update()
{
  // update controller inputs and ouputs
  update_controller_inputs();
  m_master.send_sync();
  update_controller_outputs();
}

void slug_thread::exit()
{
  // close CAN channel and clean up
  m_channel.close_channel();
  for (int i = 0; i < m_slug_ids.size(); i++)
  {
    delete m_slug[i];
  }
}
