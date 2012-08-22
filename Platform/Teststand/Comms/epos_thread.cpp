#include <math.h>
#include <vector>
#include <stdio.h>
#include <assert.h>
#include <pthread.h>
#include <string.h>
#include "shared_data.h"
#include "epos_thread.h"
#include "epos_settings.h"
#include "epos_slave.h"
#include "co_master.h"
#include "Kinematics.h"
#include "Statics.h"
#include "utils.h"
#include "stdio.h"

// epos_thread : epos controller communication thread for actuator teststand
// author: Mike Hopkins
///////////////////////////////////////////////////////////////////////////

#define SDO_TIMEOUT  1
#define SYNC_TIMEOUT 0.01

#define CONTROLWORD                 DSP402_CONTROLWORD, 0x00
#define MODES_OF_OPERATION          DSP402_MODES_OF_OPERATION, 0x00
#define MODES_OF_OPERATION_DISPLAY  DSP402_MODES_OF_OPERATION_DISPLAY, 0x00
#define POSITION_MODE_SETTING_VALUE EPOS_POSITION_MODE_SETTING_VALUE, 0x00
#define CURRENT_MODE_SETTING_VALUE  EPOS_CURRENT_MODE_SETTING_VALUE, 0x00
#define POSITION_ACTUAL_VALUE       DSP402_POSITION_ACTUAL_VALUE, 0x00
#define CURRENT_ACTUAL_VALUE        DSP402_CURRENT_ACTUAL_VALUE, 0x00
#define ANALOG_INPUT_1              EPOS_ANALOG_INPUTS, 0x01

using namespace shared_data;

namespace shared_data {
  extern struct actuator_data actuator;
  extern struct sensor_data sensor;
  extern struct bias_data bias;
};

epos_thread::epos_thread()
{
  m_id.resize(0);
  sprintf(m_interface, "\0");
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

    m_master.enter_preoperational_state(node_id);
    usleep(1e3);

    // reset controller
    fprintf(stderr, "node %d: resetting controller\n", node_id);
    m_epos[i]->fault_reset();
    assert(m_master.sdo_download(node_id, CONTROLWORD));
    m_epos[i]->shutdown();
    assert(m_master.sdo_download(node_id, CONTROLWORD));

    // configure object dictionary
    fprintf(stderr, "node %d: configuring controller mappings\n", node_id);
    assert(m_master.configure_pdo_settings(node_id));
    fprintf(stderr, "node %d: configuring controller settings\n", node_id);
    assert(m_master.configure_sdo_settings(node_id, epos_default_settings));
    assert(m_master.configure_sdo_settings(node_id, epos_custom_settings[motor_id]));
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
    bool homing_attained = true;
    for (int i = 0; i < m_id.size(); i++)
      homing_attained &= (m_epos[i]->get_statusbit_homing_attained() == 1);
    if (homing_attained)
      break;
  }
}

void epos_thread::update_actuator_settings()
{

  /************* TEMPORARY JOINT TO MOTOR CONVERSIONS ***************/

  // apply joint bias values
  std::vector<double> joint_position(N_JOINT);
  std::vector<double> joint_force(N_JOINT);
  for (int i = 0; i < N_JOINT; i++)
  {
    joint_position[i] = actuator.joint_position[i];
    joint_position[i] += bias.joint_position[i];
    joint_force[i] = actuator.joint_force[i]; 
    joint_force[i] += bias.joint_force[i];
  }

  // convert joint units to motor units
  std::vector<double> motor_position =
    kinematics_inverse_joints(&joint_position[0]);
  std::vector<double> motor_force =
    statics_inverse_joints(&joint_force[0], &joint_position[0], &motor_position[0]);

  // apply motor unit conversions
  for (int i = 0; i < N_MOTOR; i++)
  {
    motor_position[i] *= motor_position_sign[i]*motor_position_ratio[i];
    motor_position[i] += bias.motor_position[i];
    motor_force[i] *= motor_force_sign[i]*motor_force_ratio[i];
    motor_force[i] += bias.motor_force[i];
  }

  /******************************************************************/

  // send actuator settings 
  for (int i = 0; i < m_id.size(); i++)
  {
    int node_id = m_epos[i]->get_node_id();
    int motor_id = m_id[i];
    int joint_id = m_id[i];

    // set controller setpoints
    m_epos[i]->set_value(POSITION_MODE_SETTING_VALUE, motor_position[motor_id]);

    // not updating CURRENT_MODE_SETTING_VALUE for safety reasons

    // set controller enable 
    int enable = m_epos[i]->get_statusbit_operation_enabled();
    if ((actuator.joint_enable_updated[joint_id] == 1)
    || ((int)actuator.joint_enable[joint_id] != enable))
    {
      actuator.joint_enable_updated[joint_id] = 0;
      switch ((int)actuator.joint_enable[joint_id])
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

    m_master.send_rpdo(node_id, 1);

    // set controller mode
    int mode = m_epos[i]->get_value(MODES_OF_OPERATION_DISPLAY);
    if ((actuator.joint_mode_updated[joint_id] == 1)
    || (((int)actuator.joint_mode[joint_id] == 0) && (mode != -1))
    || (((int)actuator.joint_mode[joint_id] == 1) && (mode != -3)))
    {
      actuator.joint_mode_updated[joint_id] = 0; 
      switch ((int)actuator.joint_mode[joint_id])
      {
        case 0 :
          m_epos[i]->set_value(MODES_OF_OPERATION, -1);
          m_epos[i]->set_controlbit_new_setpoint(1);
          m_epos[i]->set_controlbit_change_set_immediately(1);
          break;
        case 1 :
          m_epos[i]->set_value(MODES_OF_OPERATION, -3);
          m_epos[i]->set_controlbit_new_setpoint(1);
          m_epos[i]->set_controlbit_change_set_immediately(1);
          break;
        default :
          break;
      }
      m_master.send_rpdo(node_id, 4);
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

  // update sensor values
  for (int i = 0; i < m_id.size(); i++)
  {
    int motor_id = m_id[i];
    sensor.motor_position[motor_id] = m_epos[i]->get_value(POSITION_ACTUAL_VALUE)
      - bias.motor_position[motor_id];
    sensor.motor_force[motor_id] = m_epos[i]->get_value(ANALOG_INPUT_1)
      - bias.motor_force[motor_id];
    sensor.motor_current[motor_id] = m_epos[i]->get_value(CURRENT_ACTUAL_VALUE);
  }

  /************* TEMPORARY MOTOR TO JOINT CONVERSIONS ***************/

  // apply motor unit conversions
  std::vector<double> motor_position(N_MOTOR);
  std::vector<double> motor_force(N_MOTOR);
  for (int i = 0; i < N_MOTOR; i++)
  {
    motor_position[i] = sensor.motor_position[i];
    motor_position[i] /= motor_position_sign[i]*motor_position_ratio[i];
    motor_force[i] = sensor.motor_force[i];
    motor_force[i] /= motor_force_sign[i]*motor_force_ratio[i];
  }

  // convert motor units to joint units 
  std::vector<double> joint_position =
    kinematics_forward_joints(&motor_position[0]);
  std::vector<double> joint_force =
    statics_forward_joints(&motor_force[0], &joint_position[0], &motor_position[0]);

  // apply joint bias values 
  for (int i = 0; i < m_id.size(); i++)
  {
    int joint_id = m_id[i];
    sensor.joint_position[joint_id] = joint_position[joint_id]
      - bias.joint_position[joint_id];
    sensor.joint_force[joint_id] = joint_force[joint_id]
      - bias.joint_force[joint_id];
  }  

  /******************************************************************/

}

void epos_thread::entry()
{
  // initialize can bus
  assert(0 == m_channel.open_channel(m_interface));

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
  //home_motor_controllers();

  // enable position control
  m_master.send_sync(0);
  update_sensor_readings();
  for (int i = 0; i < m_id.size(); i++) 
  {
    int joint_id = m_id[i];
    actuator.joint_mode[joint_id] = 0;
    actuator.joint_enable[joint_id] = 1;
    actuator.joint_position[joint_id] = sensor.joint_position[joint_id];
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

void epos_thread::set_joints(std::vector<int> id)
{
  m_id = id;
}

void epos_thread::set_interface(const char *interface)
{
  sprintf(m_interface, "%s", interface);
}
