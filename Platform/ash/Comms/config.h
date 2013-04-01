#ifndef _CONFIG_H_
#define _CONFIG_H_

#include <vector>

// config : global interface for config data
// author : Mike Hopkins
///////////////////////////////////////////////////////////////////////////

// define device array lengths
#define N_JOINT 31
#define N_MOTOR 31
#define N_FORCE_TORQUE 24
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
  Config();
  ~Config();
};

#endif
