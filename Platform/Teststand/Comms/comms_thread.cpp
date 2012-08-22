#include <pthread.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "comms_thread.h"
#include "utils.h"

// comms_thread : base class for active communication objects
// author: Mike Hopkins
///////////////////////////////////////////////////////////////////////////

#define FPS_UPDATE 20

comms_thread::comms_thread()
{
  m_thread = 0;
  m_is_running = 0;
  m_stop_request = 0;
  sprintf(m_error_message, '\0');
  m_fps = 0;
}

void comms_thread::entry()
{

}

void comms_thread::update()
{

}

void comms_thread::exit()
{

}

void *comms_thread::thread_func(void *args)
{
  comms_thread *this_comms_thread = (comms_thread *)args; 

  this_comms_thread->entry();
  this_comms_thread->m_is_running = 1;

  long count = 0;
  double t0 = get_time();
  while (!this_comms_thread->m_stop_request)
  {
    if (0 == (++count % FPS_UPDATE))
    {
      double t = get_time();
      this_comms_thread->m_fps = FPS_UPDATE/(t - t0);
      t0 = t;
    }
    this_comms_thread->update();
  }

  this_comms_thread->m_is_running = 0;
  this_comms_thread->exit();
  return NULL;
}

int comms_thread::is_running()
{
  return m_is_running;
}

double comms_thread::get_fps()
{
  return m_fps;
}

char *comms_thread::get_error_message()
{
  return m_error_message; 
}

void comms_thread::get_thread(pthread_t *thread)
{
  *thread = m_thread;
}

void comms_thread::start()
{
  m_stop_request = 0;
  if (0 != pthread_create(&m_thread, NULL, thread_func, this))
  {
    perror("failed to create thread\n");
  }
}

void comms_thread::stop()
{
  m_stop_request = 1;
  if (m_thread)
    pthread_join(m_thread, NULL);
}
