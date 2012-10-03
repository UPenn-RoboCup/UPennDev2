#ifndef _DCM_H_
#define _DCM_H_

#include "config.h"

// dcm : global interface for sensor and actuator data
// author : Mike Hopkins
///////////////////////////////////////////////////////////////////////////

class Dcm {
public:
  // shared data
  double *joint_enable;
  double *joint_stiffness;
  double *joint_damping;
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
  double *tactile_array;
  double *ahrs;
  double *battery;
  // update flags
  double *joint_enable_updated;
  double *joint_stiffness_updated;
  double *joint_damping_updated;
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
  double *tactile_array_updated;
  double *ahrs_updated;
  double *battery_updated;
  Dcm();
  ~Dcm();
};

#endif
