#ifndef _DCM_H_
#define _DCM_H_

#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/allocators/allocator.hpp>
#include <boost/interprocess/exceptions.hpp>
#include <vector>
#include <string>
#include "config.h"

// dcm : global interface for sensor and actuator data
// author : Mike Hopkins
///////////////////////////////////////////////////////////////////////////

using namespace boost::interprocess;

class Dcm {
public:
  // shared data
  double *joint_enable;
  double *joint_p_gain;
  double *joint_i_gain;
  double *joint_d_gain;
  double *joint_force;
  double *joint_position;
  double *joint_velocity;
  double *joint_force_sensor;
  double *joint_position_sensor;
  double *joint_velocity_sensor;
  double *motor_force_sensor;
  double *motor_position_sensor;
  double *motor_velocity_sensor;
  double *motor_current_sensor;
  double *motor_temperature_sensor;
  double *force_torque;
  double *ahrs;
  double *battery;
  // update flags
  double *joint_enable_updated;
  double *joint_p_gain_updated;
  double *joint_i_gain_updated;
  double *joint_d_gain_updated;
  double *joint_force_updated;
  double *joint_position_updated;
  double *joint_velocity_updated;
  double *joint_force_sensor_updated;
  double *joint_position_sensor_updated;
  double *joint_velocity_sensor_updated;
  double *motor_force_sensor_updated;
  double *motor_position_sensor_updated;
  double *motor_velocity_sensor_updated;
  double *motor_current_sensor_updated;
  double *motor_temperature_sensor_updated;
  double *force_torque_updated;
  double *ahrs_updated;
  double *battery_updated;
  // data lengths
  int n_joint;
  int n_motor;
  int n_force_torque;
  int n_ahrs;
  int n_battery;
  Dcm();
  ~Dcm();
private:
  managed_shared_memory dcm_segment;
};

#endif
