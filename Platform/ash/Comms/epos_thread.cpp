#include <math.h>
#include <vector>
#include <stdio.h>
#include <assert.h>
#include <unistd.h>
#include <pthread.h>
#include <string.h>
#include "epos_thread.h"
#include "epos_settings.h"
#include "epos_slave.h"
#include "co_master.h"
#include "joint_kinematics.h"
#include "joint_statics.h"
#include "config.h"
#include "utils.h"
#include "dcm.h"

// epos_thread : epos controller communication thread for ASH
// author : Mike Hopkins
///////////////////////////////////////////////////////////////////////////

#define SDO_TIMEOUT  1
#define SYNC_TIMEOUT 0.01

#define CONTROLWORD                 DSP402_CONTROLWORD, 0x00
#define MODES_OF_OPERATION          DSP402_MODES_OF_OPERATION, 0x00
#define MODES_OF_OPERATION_DISPLAY  DSP402_MODES_OF_OPERATION_DISPLAY, 0x00
#define POSITION_MODE_SETTING_VALUE EPOS_POSITION_MODE_SETTING_VALUE, 0x00
#define TARGET_TORQUE               EPOS_TARGET_TORQUE, 0X00
#define POSITION_ACTUAL_VALUE       DSP402_POSITION_ACTUAL_VALUE, 0x00
#define CURRENT_ACTUAL_VALUE        DSP402_CURRENT_ACTUAL_VALUE, 0x00
#define TORQUE_ACTUAL               EPOS_TORQUE_ACTUAL, 0x00

extern Dcm dcm;
extern Config config;

epos_thread::epos_thread()
{
  m_id.resize(0);
  sprintf(m_can_interface, "\0");
}

void epos_thread::set_joints(std::vector<int> id)
{
  m_id = id;
}

void epos_thread::set_can_interface(const char *interface)
{
  strncpy(m_can_interface, interface, 128);
}

bool epos_thread::check_joint_settings()
{
  bool joint_enable_valid = 
    (dcm.joint_enable[0] == dcm.joint_enable[1] == 
     dcm.joint_enable[2]) &&
    (dcm.joint_enable[4] == dcm.joint_enable[5]) &&
    (dcm.joint_enable[6] == dcm.joint_enable[7] == 
     dcm.joint_enable[8]) &&
    (dcm.joint_enable[10] == dcm.joint_enable[11]);
  bool joint_position_p_gain_valid = 
    (dcm.joint_position_p_gain[0] == dcm.joint_position_p_gain[1] == 
     dcm.joint_position_p_gain[2]) &&
    (dcm.joint_position_p_gain[4] == dcm.joint_position_p_gain[5]) &&
    (dcm.joint_position_p_gain[6] == dcm.joint_position_p_gain[7] ==
     dcm.joint_position_p_gain[8]) &&
    (dcm.joint_position_p_gain[10] == dcm.joint_position_p_gain[11]);
  return joint_enable_valid && joint_position_p_gain_valid;
}

void epos_thread::emcy_callback(int node_id, void *user_data)
{
  // terminate comms manager upon receiving any emergency messages
  epos_thread *this_epos_thread = (epos_thread *)user_data;

  int error_code = 0;
  for (int i = 0; i < this_epos_thread->m_id.size(); i++)
  {
    if (node_id == this_epos_thread->m_epos[i]->get_node_id())
      error_code = this_epos_thread->m_epos[i]->get_error_code();
  }

  sprintf(this_epos_thread->m_error_message, "node %d: emcy error %x",
    node_id, error_code);

  this_epos_thread->m_stop_request = 1;
}

void epos_thread::initialize_motor_controllers()
{
  // initialize epos controllers
  for (int i = 0; i < m_id.size(); i++)
  {
    int node_id = m_epos[i]->get_node_id(); 
    int motor_id = m_id[i];
    double torque_feedback_offset;
    double torque_feedback_scaling;
    double home_offset, position_offset;

    m_master.enter_preoperational_state(node_id);
    usleep(1e3);

    // reset controller
    fprintf(stderr, "node %d: resetting controller\n", node_id);
    m_epos[i]->fault_reset();
    assert(m_master.sdo_download(node_id, CONTROLWORD));
    m_epos[i]->shutdown();
    assert(m_master.sdo_download(node_id, CONTROLWORD));

    // hack for torque mode analog input configuration bug
    if (i > 0)
    {
      m_epos[i]->set_value(MODES_OF_OPERATION, -32);
      m_master.sdo_download(node_id, MODES_OF_OPERATION);
    }

    // configure object dictionary
    fprintf(stderr, "node %d: configuring controller mappings\n", node_id);
    m_master.configure_pdo_settings(node_id);
    fprintf(stderr, "node %d: configuring controller settings\n", node_id);
    assert(m_master.configure_sdo_settings(node_id, epos_default_settings));
    assert(m_master.configure_sdo_settings(node_id, epos_custom_settings[motor_id]));

    // configure position bias
    fprintf(stderr, "node %d: configuring position bias\n", node_id);
    home_offset = m_epos[i]->get_value(
      DSP402_HOME_OFFSET, 0x00); 
    position_offset  = config.motor_position_bias[motor_id];
    position_offset *= motor_position_sign[motor_id]*motor_position_ratio[motor_id];
    m_epos[i]->set_value(
      DSP402_HOME_OFFSET, 0x00, home_offset + position_offset);
    assert(m_master.sdo_download(node_id,
      DSP402_HOME_OFFSET, 0x00, SDO_TIMEOUT));

    // configure torque bias
    fprintf(stderr, "node %d: configuring torque bias\n", node_id);
    torque_feedback_scaling = m_epos[i]->get_value(
      EPOS_ANALOG_TORQUE_FEEDBACK_CONFIGURATION, 0x01);
    torque_feedback_offset  = -config.motor_force_bias[motor_id];
    torque_feedback_offset *= motor_force_sign[motor_id]*motor_force_ratio[motor_id];
    torque_feedback_offset /= torque_feedback_scaling;
    m_epos[i]->set_value(
      EPOS_ANALOG_TORQUE_FEEDBACK_CONFIGURATION, 0x02, torque_feedback_offset);
    m_master.sdo_download(node_id,
      EPOS_ANALOG_TORQUE_FEEDBACK_CONFIGURATION, 0x02, SDO_TIMEOUT);
    
    // store object dictionary
    assert(m_master.store_all_parameters(node_id));

    usleep(1e3);
    m_master.start_remote_node(node_id);
  }
}

void epos_thread::home_motor_controllers()
{
  // get transmit can_ids for sync polling
  co_can_id sync_ids[128];
  for (int i = 0; i < m_id.size(); i++)
  {
    int node_id = m_epos[i]->get_node_id();
    sync_ids[i] = CO_TPDO1 | node_id;
  }

  // start homing operation
  for (int i = 0; i < m_id.size(); i++)
  {
    int node_id = m_epos[i]->get_node_id();
    fprintf(stderr, "node %d: starting homing\n", node_id);
    m_epos[i]->set_value(MODES_OF_OPERATION, 6);
    m_master.sdo_download(node_id, MODES_OF_OPERATION);
    m_epos[i]->enable_operation();
    m_master.sdo_download(node_id, CONTROLWORD);
    usleep(1e4);
    m_epos[i]->set_controlbit_homing_operation_start(1);
    m_master.sdo_download(node_id, CONTROLWORD);
  }

  // wait for homing attained status
  while (1)
  {
    m_master.send_sync(); 
    m_master.receive(sync_ids, m_id.size(), SYNC_TIMEOUT); 
    int homing_attained = 0;
    for (int i = 0; i < m_id.size(); i++)
      homing_attained += m_epos[i]->get_statusbit_homing_attained();
    if (homing_attained == m_id.size())
      break;
  }
  fprintf(stderr, "homing done!\n");
}

void epos_thread::update_actuator_settings()
{

  // apply joint bias settings
  std::vector<double> joint_position_sensor(N_JOINT);
  std::vector<double> joint_position(N_JOINT);
  std::vector<double> joint_force(N_JOINT);
  for (int i = 0; i < N_JOINT; i++)
  {
    joint_position_sensor[i] = dcm.joint_position_sensor[i]
      + config.joint_position_bias[i];
    joint_position[i] = dcm.joint_position[i]
      + config.joint_position_bias[i];
    joint_force[i] = dcm.joint_force[i]
      + config.joint_force_bias[i];
  }

  // convert joint units to motor units
  std::vector<double> motor_position =
    kinematics_inverse_joints(&joint_position[0]);
  std::vector<double> motor_force =
    statics_inverse_joints(&joint_force[0], 
    &joint_position_sensor[0], dcm.motor_position_sensor);

  // gaurd against invalid joint settings
  if (!check_joint_settings())
  {
    // sprintf(m_error_message, "invalid joint settings");
    // m_stop_request = 1;
  }

  // send actuator settings 
  for (int i = 0; i < m_id.size(); i++)
  {
    int node_id = m_epos[i]->get_node_id();
    int motor_id = m_id[i];
    int joint_id = m_id[i];

    // update controller enable 
    int enable = m_epos[i]->get_statusbit_operation_enabled();
    if ((dcm.joint_enable_updated[joint_id] == 1)
    || ((int)dcm.joint_enable[joint_id] != enable))
    {
      dcm.joint_enable_updated[joint_id] = 0;
      switch ((int)dcm.joint_enable[joint_id])
      {
        case 0 :
          m_epos[i]->shutdown();
          break;
        case 1 :
          m_epos[i]->enable_operation();
        default :
          break;
      }
    }

    // update setpoints
    motor_position[motor_id] *= motor_position_sign[motor_id];
    motor_position[motor_id] *= motor_position_ratio[motor_id];
    motor_force[motor_id] *= motor_force_sign[motor_id];
    motor_force[motor_id] *= motor_force_ratio[motor_id];
    m_epos[i]->set_value(POSITION_MODE_SETTING_VALUE, motor_position[motor_id]);
    m_epos[i]->set_value(TARGET_TORQUE, motor_force[motor_id]);
    m_epos[i]->set_controlbit_new_setpoint(1);
    m_epos[i]->set_controlbit_change_set_immediately(1);
    if ((int)dcm.joint_position_p_gain[joint_id] == 0)
      m_master.send_rpdo(node_id, 2);
    else
      m_master.send_rpdo(node_id, 1);

    // update controller mode
    int mode = m_epos[i]->get_value(MODES_OF_OPERATION_DISPLAY);
    if ((dcm.joint_position_p_gain_updated[joint_id] == 1)
    || (((int)dcm.joint_position_p_gain[joint_id] == 1) && (mode != -1))
    || (((int)dcm.joint_position_p_gain[joint_id] == 0) && (mode != -32)))
    {
      dcm.joint_position_p_gain_updated[joint_id] = 0; 
      switch ((int)dcm.joint_position_p_gain[joint_id])
      {
        case 1 :
          m_epos[i]->set_value(MODES_OF_OPERATION, -1);
          m_master.send_rpdo(node_id, 4);
          break;
        case 0 :
          m_epos[i]->set_value(MODES_OF_OPERATION, -32);
          m_master.send_rpdo(node_id, 4);
          break;
        default :
          break;
      }
    }
  }
}

void epos_thread::update_sensor_readings()
{
  // get transmit can_ids for sync polling
  co_can_id sync_ids[128];
  for (int i = 0; i < m_id.size(); i++)
  {
    int node_id = m_epos[i]->get_node_id();
    sync_ids[i] = CO_TPDO1 | node_id;
  }

  // receive sync data 
  m_master.receive(sync_ids, m_id.size(), SYNC_TIMEOUT);
  m_master.flush_can_channel();

  // update motor sensor values
  for (int i = 0; i < m_id.size(); i++)
  {
    int motor_id = m_id[i];
    double motor_force, motor_position;
    double torque_feedback_scaling = m_epos[i]->get_value(
      EPOS_ANALOG_TORQUE_FEEDBACK_CONFIGURATION,  0x01);
    double torque_feedback_offset = m_epos[i]->get_value(
      EPOS_ANALOG_TORQUE_FEEDBACK_CONFIGURATION,  0x02);
    motor_force  = m_epos[i]->get_value(EPOS_ANALOG_INPUTS, 0x01);
    motor_force += torque_feedback_offset;
    motor_force *= torque_feedback_scaling;
    motor_force /= motor_force_sign[motor_id]*motor_force_ratio[motor_id];
    motor_position  = m_epos[i]->get_value(POSITION_ACTUAL_VALUE);
    motor_position /= motor_position_sign[motor_id]*motor_position_ratio[motor_id];
    dcm.motor_force_sensor[motor_id] = motor_force;
    dcm.motor_position_sensor[motor_id] = motor_position;
    dcm.motor_current_sensor[motor_id] = m_epos[i]->get_value(CURRENT_ACTUAL_VALUE);
  }

  // convert motor units to joint units
  std::vector<double> joint_position =
    kinematics_forward_joints(dcm.motor_position_sensor);
  std::vector<double> joint_force =
    statics_forward_joints(dcm.motor_force_sensor,
    &joint_position[0], dcm.motor_position_sensor);

  // apply joint bias settings 
  for (int i = 0; i < m_id.size(); i++)
  {
    int joint_id = m_id[i];
    dcm.joint_position_sensor[joint_id] = joint_position[joint_id]
      - config.joint_position_bias[joint_id];
    dcm.joint_force_sensor[joint_id] = joint_force[joint_id]
      - config.joint_force_bias[joint_id];
  }  

}

void epos_thread::entry()
{
  // initialize can bus
  assert(0 == m_channel.open_channel(m_can_interface));

  // initialize epos slave objects
  for (int i = 0; i < m_id.size(); i++)
  {
    int motor_id = m_id[i];
    m_epos[i] = new epos_slave();
    m_epos[i]->set_node_id(epos_node_id[motor_id]);
    m_epos[i]->register_sdo_settings(epos_default_settings);
    m_epos[i]->register_sdo_settings(epos_custom_settings[motor_id]);
    m_epos[i]->register_pdo_parameters(epos_pdo_parameters);
    m_epos[i]->register_pdo_mappings(epos_pdo_mappings);
  }

  // initialize CANopen master
  m_master.set_can_channel(&m_channel);
  m_master.enable_debug_print(true);
  m_master.set_default_timeout(SDO_TIMEOUT);
  for (int i = 0; i < m_id.size(); i++)
  {
    m_master.register_slave(m_epos[i]);
  }

  // initialize motor controllers
  initialize_motor_controllers();
 
  // enable emergency message handler
  m_master.register_emcy_callback(&epos_thread::emcy_callback, this);

  // home_motor_controllers
  home_motor_controllers();

  // enable joints in position mode
  m_master.send_sync(0);
  update_sensor_readings();
  for (int i = 0; i < m_id.size(); i++) 
  {
    int joint_id = m_id[i];
    dcm.joint_enable[joint_id] = 1;
    dcm.joint_position_p_gain[joint_id] = 1;
    dcm.joint_position_i_gain[joint_id] = 0;
    dcm.joint_position_d_gain[joint_id] = 0;
    dcm.joint_velocity_p_gain[joint_id] = 0;
    dcm.joint_force[joint_id] = 0;
    dcm.joint_position[joint_id] = dcm.joint_position_sensor[joint_id];
    dcm.joint_velocity[joint_id] = 0; 
  }
}

void epos_thread::update()
{
  // update main communications loop
  update_actuator_settings();
  m_master.send_sync(0);
  update_sensor_readings();
}

void epos_thread::exit()
{
  // close can bus and clean up
  m_channel.close_channel();
  for (int i = 0; i < m_id.size(); i++)
  {
    delete m_epos[i];
  }
}
