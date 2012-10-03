#ifndef SENSOR_THREAD_H
#define SENSOR_THREAD_H

#include "comms_thread.h"
#include "Microstrain.h"
#include "netft_node.h"
#include "co_master.h"

// sensor_thread : motion sensor communication thread for ASH
// author : Mike Hopkins
///////////////////////////////////////////////////////////////////////////

class sensor_thread : public comms_thread {
private:
  char m_imu_interface[128];
  char m_can_interface[128];
  Microstrain m_imu;
  netft_node m_netft[2];
  can_channel m_channel;
  co_master m_master;
  virtual void entry();
  virtual void update();
  virtual void exit();
public:
  sensor_thread();
  void set_can_interface(const char *interface);
  void set_imu_interface(const char *interface);
};

#endif
