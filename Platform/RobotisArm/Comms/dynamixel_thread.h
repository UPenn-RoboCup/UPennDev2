#ifndef DYNAMIXEL_THREAD_H
#define DYNAMIXEL_THREAD_H

#include <vector>
#include "comms_thread.h"
#include "dynamixel.h"

// dynamixel_thead : serial controller communication thread for robotis arm
// author(s) : 
///////////////////////////////////////////////////////////////////////////

class dynamixel_thread : public comms_thread {
protected:
  SerialPort sp;
  SerialPort *Port;  
  std::vector<int> m_id;
  char m_interface[128];
  void home_motor_controllers();
  void update_actuator_settings();
  void update_sensor_readings();
  virtual void entry();
  virtual void update();
  virtual void exit();
public:
  dynamixel_thread();
  void set_joints(std::vector<int> id);
  void set_interface(const char *interface);
};

#endif
