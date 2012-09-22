#ifndef EPOS_THREAD_H
#define EPOS_THREAD_H

#include <vector>
#include "co_master.h"
#include "epos_slave.h"
#include "comms_thread.h"

// epos_thread : epos controller communication thread for ASH
// author: Mike Hopkins
///////////////////////////////////////////////////////////////////////////

class epos_thread : public comms_thread {
protected:
  std::vector<int> m_id;
  char m_can_interface[128];
  epos_slave *m_epos[128];
  can_channel m_channel;
  co_master m_master;
  bool check_joint_settings();
  static void emcy_callback(int node_id, void *user_data);
  void initialize_motor_controllers();
  void home_motor_controllers();
  void update_actuator_settings();
  void update_sensor_readings();
  virtual void entry();
  virtual void update();
  virtual void exit();
public:
  epos_thread();
  void set_joints(std::vector<int> id);
  void set_can_interface(const char *interface);
};

#endif
