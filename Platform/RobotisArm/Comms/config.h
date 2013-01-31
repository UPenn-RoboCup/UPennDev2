#ifndef _CONFIG_H_
#define _CONFIG_H_

#include <vector>

// config : global interface for config data
// author : Mike Hopkins
///////////////////////////////////////////////////////////////////////////

// define device array lengths
#define N_JOINT 9
#define N_MOTOR 9
#define N_FORCE_TORQUE 12 
#define N_TACTILE_ARRAY 6
#define N_AHRS 9 
#define N_BATTERY 3

class Config {
public:
  // config data
  std::vector<double> joint_force_bias;
  std::vector<double> joint_position_bias;
  std::vector<double> motor_force_bias;
  std::vector<double> motor_position_bias;
  std::vector<double> force_torque_bias;
  std::vector<double> tactile_array_bias;
  Config();
  ~Config();
};

#endif
