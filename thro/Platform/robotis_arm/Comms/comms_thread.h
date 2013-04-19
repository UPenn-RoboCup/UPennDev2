#ifndef COMMS_THREAD_H
#define COMMS_THREAD_H

#include <pthread.h>

// comms_thread : base class for active communication objects
// author : Mike Hopkins
///////////////////////////////////////////////////////////////////////////

class comms_thread {
protected:
  int m_is_running;
  int m_stop_request;
  char m_error_message[128];
  pthread_t m_thread;
  double m_fps;
  static void *thread_func(void *args);
  virtual void entry();
  virtual void update();
  virtual void exit();
public:
  comms_thread();
  int is_running();
  char *get_error_message();
  void get_thread(pthread_t *thread);
  double get_fps();
  void start();
  void stop();
};

#endif
