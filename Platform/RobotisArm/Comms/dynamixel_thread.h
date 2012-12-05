#ifndef DYNAMIXEL_THREAD_H
#define DYNAMIXEL_THREAD_H

#include <vector>
#include "comms_thread.h"

// dynamixel_thead : epos controller communication thread for robotis arm
// author(s) : 
///////////////////////////////////////////////////////////////////////////

class dynamixel_thread : public comms_thread {
protected:
  std::vector<int> m_id;
  char m_interface[128];
  virtual void entry();
  virtual void update();
  virtual void exit();
public:
  dynamixel_thread();
  void set_joints(std::vector<int> id);
  void set_interface(const char *interface);
};

#endif
