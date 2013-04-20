#include "slug_thread.h"
#include "config.h"
#include "dcm.h"

// slug_thread : motor slug communication thread for teststand
// author : Mike Hopkins
///////////////////////////////////////////////////////////////////////////

extern Dcm dcm;
extern Config config;

slug_thread::slug_thread()
{
  sprintf(m_can_interface, "\0");
}

void slug_thread::set_can_interface(const char *interface)
{
  strncpy(m_can_interface, interface, 128);
}

void slug_thread::emcy_callback(int node_id, void *user_data)
{
  // terminate comms manager upon receiving an emergency message
  int error_code = 0;
  slug_thread *this_slug_thread = (slug_thread *)user_data;
  if (node_id == this_slug_thread->m_slug[0]->get_node_id())
    error_code = this_slug_thread->m_slug[0]->get_error_code();
  sprintf(this_slug_thread->m_error_message,
    "node %d: emcy error %x",
    node_id, error_code
  );
  this_slug_thread->m_stop_request = 1;
}

void slug_thread::initialize_controllers()
{

}

void slug_thread::update_controller_inputs()
{

}

void slug_thread::update_controller_outputs()
{

}

void slug_thread::entry()
{

}

void slug_thread::update()
{

}

void slug_thread::exit()
{

}
