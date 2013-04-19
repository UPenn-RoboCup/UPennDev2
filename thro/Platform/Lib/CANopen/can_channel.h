#ifndef CAN_CHANNEL_H
#define CAN_CHANNEL_H

/*
 * can_channel : driver interface for managing CAN bus communication
 * author : Mike Hopkins
 */

#include "co_types.h"

class can_channel {
private:
  int m_fd; 
public:
  can_channel();
  can_channel(const char *interface_name);
  int get_fd();
  int open_channel(const char *interface_name);
  int set_loopback(int enable);
  int get_loopback();
  int set_timeout(double timeout);
  double get_timeout();
  int send(struct can_frame *frame);
  int receive(struct can_frame *frame);
  void flush();
  void close_channel();
  ~can_channel();
};

#endif
