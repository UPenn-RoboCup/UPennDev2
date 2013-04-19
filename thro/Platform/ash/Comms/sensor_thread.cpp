#include <math.h>
#include <string.h>
#include "sensor_thread.h"
#include "dcm.h"
#include "config.h"
#include <assert.h>

// sensor_thread : motion sensor communication thread for ASH
// author : Mike Hopkins
///////////////////////////////////////////////////////////////////////////

extern Dcm dcm;
extern Config config;

#define L_FT_NODE_ID             8
#define R_FT_NODE_ID             16
#define L_FT_FORCE_SCALE_FACTOR  44253 
#define R_FT_FORCE_SCALE_FACTOR  44253 
#define L_FT_TORQUE_SCALE_FACTOR 763
#define R_FT_TORQUE_SCALE_FACTOR 763
#define CAN_TIMEOUT              0.01

sensor_thread::sensor_thread()
{
  sprintf(m_can_interface, "\0");
  sprintf(m_imu_interface, "\0");
}

void sensor_thread::set_can_interface(const char *interface)
{
  strncpy(m_can_interface, interface, 128);
}

void sensor_thread::set_imu_interface(const char *interface)
{
  strncpy(m_imu_interface, interface, 128);
}

void sensor_thread::entry()
{
  // initialize imu
  double gyro_bias[3] = {0, 0, 0};
  m_imu.openPort(m_imu_interface);
  m_imu.initTime(0);
//m_imu.initGyros(&gyro_bias[0], &gyro_bias[1], &gyro_bias[2]);

  // initialize can bus
  assert(0 == m_channel.open_channel(m_can_interface));

  // initialize can master
  m_master.set_can_channel(&m_channel);
  m_master.enable_debug_print(true);
 
  // initialize force-torque nodes
  m_netft[0].set_node_id(L_FT_NODE_ID);
  m_netft[1].set_node_id(R_FT_NODE_ID);
  m_netft[0].set_force_scale_factor(L_FT_FORCE_SCALE_FACTOR);
  m_netft[1].set_force_scale_factor(R_FT_FORCE_SCALE_FACTOR);
  m_netft[0].set_torque_scale_factor(L_FT_TORQUE_SCALE_FACTOR);
  m_netft[1].set_torque_scale_factor(R_FT_TORQUE_SCALE_FACTOR);

  std::vector<struct can_frame> &l_ft_buffers = m_netft[0].get_can_buffers();
  std::vector<struct can_frame> &r_ft_buffers = m_netft[1].get_can_buffers();
  for (int i = 0; i < l_ft_buffers.size(); i++)
    m_master.register_can_buffer(&l_ft_buffers[i]);
  for (int i = 0; i < r_ft_buffers.size(); i++)
    m_master.register_can_buffer(&r_ft_buffers[i]);
}

void sensor_thread::update()
{
    // request force-torque data
    struct can_frame frame;
    frame = m_netft[0].get_short_request();
    m_master.send_can_frame(&frame);
    frame = m_netft[1].get_short_request();
    m_master.send_can_frame(&frame);

    // request imu data
    uint8_t cmd[] = {Microstrain::CMD_ACCEL_ANGRATE_ORIENT};
    m_imu.send(cmd, sizeof(cmd));

    // receive force-torque data
    co_can_id receive_ids[] = {
      L_FT_NODE_ID + 5,
      L_FT_NODE_ID + 6,
      R_FT_NODE_ID + 5,
      R_FT_NODE_ID + 6
    };
    m_master.receive(receive_ids, 4, CAN_TIMEOUT);
    double l_forces[3];
    double r_forces[3];
    double l_torques[3];
    double r_torques[3];
    m_netft[0].update_data();
    m_netft[1].update_data();
    m_netft[0].get_forces(l_forces);
    m_netft[1].get_forces(r_forces);
    m_netft[0].get_torques(l_torques);
    m_netft[1].get_torques(r_torques);
    for (int i = 0; i < 3; i++)
     dcm.force_torque[i] = l_forces[i] - config.force_torque_bias[i];
    for (int i = 0; i < 3; i++)
     dcm.force_torque[i+3] = l_torques[i] - config.force_torque_bias[i+3];
    for (int i = 0; i < 3; i++)
     dcm.force_torque[i+6] = r_forces[i] - config.force_torque_bias[i+6];
    for (int i = 0; i < 3; i++)
     dcm.force_torque[i+9] = r_torques[i] - config.force_torque_bias[i+9];

    // receive imu data
    uint64_t time;
    double accel[3], gyro[3], orientation[9];
    m_imu.receiveAccelAngrateOrientation(&time, accel, gyro, orientation);
    dcm.ahrs[0] = accel[1];                              // x accel
    dcm.ahrs[1] = accel[0];                              // y accel
    dcm.ahrs[2] = -accel[2];                             // z accel
    dcm.ahrs[3] = gyro[1];                               // x gyro
    dcm.ahrs[4] = gyro[0];                               // y gyro
    dcm.ahrs[5] = -gyro[2];                              // z gyro
    dcm.ahrs[6] = -asin(orientation[2]);                 // x euler
    dcm.ahrs[7] = atan(orientation[5]/orientation[8]);   // y euler
    dcm.ahrs[8] = -atan2(orientation[1],orientation[0]); // z euler
}

void sensor_thread::exit()
{
  m_imu.closePort();
  m_channel.close_channel();
}
