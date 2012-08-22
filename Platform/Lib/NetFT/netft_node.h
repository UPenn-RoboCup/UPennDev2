#ifndef NETFT_NODE_H
#define NETFT_NODE_H

/*
 * netft_node : CAN communication interface for ATI net-ft boards 
 * author : Mike Hopkins
 */

#include "co_types.h"
#include <vector>

class netft_node {
private:
  co_can_id m_node_id;
  std::vector<struct can_frame> m_can_buffers;
  int m_forces[3];
  int m_torques[3];
  int m_status;
  int m_sample;
public:
  netft_node(co_can_id node_id = 1);
  co_can_id get_node_id();
  void set_node_id(co_can_id node_id);
  std::vector<struct can_frame> &get_can_buffers();
  struct can_frame get_long_request();
  struct can_frame get_short_request();
  struct can_frame get_bias_command();
  struct can_frame get_clear_threshold_command();
  void get_forces(int forces[3]);
  void get_torques(int torques[3]);
  int get_status();
  int get_sample();
  void update_data();
};

#endif
