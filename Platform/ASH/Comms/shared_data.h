#ifndef SHARED_DATA_H
#define SHARED_DATA_H

// shared_data : utilities for shared actuator and sensor data 
// author: Mike Hopkins
///////////////////////////////////////////////////////////////////////////

// define sensor and actuator array lengths
#define N_JOINT 12
#define N_MOTOR 12
#define N_FORCE_TORQUE 12 
#define N_TACTILE_ARRAY 6
#define N_AHRS 9 
#define N_BATTERY 3

// define structs for storing pointers to shared data
struct actuator_data {
  double *joint_write_access;
  double *joint_enable;
  double *joint_stiffness;
  double *joint_damping;
  double *joint_force;
  double *joint_position;
  double *joint_velocity;

  double *joint_write_access_updated;
  double *joint_enable_updated;
  double *joint_stiffness_updated;
  double *joint_damping_updated;
  double *joint_force_updated;
  double *joint_position_updated;
  double *joint_velocity_updated;
};

struct sensor_data {
  double *joint_force;
  double *joint_position;
  double *joint_velocity;
  double *motor_force;
  double *motor_position;
  double *motor_velocity;
  double *motor_current;
  double *motor_temperature;
  double *force_torque;
  double *tactile_array;
  double *ahrs;
  double *battery;

  double *joint_force_updated;
  double *joint_position_updated;
  double *joint_velocity_updated;
  double *motor_force_updated;
  double *motor_position_updated;
  double *motor_velocity_updated;
  double *motor_current_updated;
  double *motor_temperature_updated;
  double *force_torque_updated;
  double *tactile_array_updated;
  double *ahrs_updated;
  double *battery_updated;
};

struct bias_data { 
  double *joint_force;
  double *joint_position;
  double *motor_force;
  double *motor_position;
  double *force_torque;
  double *tactile_array;
};

namespace shared_data {
  void entry();
  void exit();
};

#endif
