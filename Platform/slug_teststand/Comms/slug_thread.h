#ifndef _SLUG_THREAD_H_
#define _SLUG_THREAD_H_

#include <vector>
#include "co_master.h"
#include "slug_slave.h"
#include "comms_thread.h"

// slug_thread : motor slug communication thread
// author : Mike Hopkins
///////////////////////////////////////////////////////////////////////////

class slug_thread : public comms_thread {
protected:
  std::vector<int> m_slug_ids;
  std::vector<int> m_node_ids;
  std::vector<const int*> m_joint_ids;
  char m_can_interface[128];
  slug_slave *m_slug[128];
  can_channel m_channel;
  co_master m_master;

  static void emcy_callback(int node_id, void *user_data);
  void initialize_controllers();
  void update_controller_inputs();
  void update_controller_outputs();

  virtual void entry();
  virtual void update();
  virtual void exit();
public:
  slug_thread();
  void set_can_interface(const char *interface);
  void set_slug_ids(int *m_ids, int n_ids);
};

#endif
