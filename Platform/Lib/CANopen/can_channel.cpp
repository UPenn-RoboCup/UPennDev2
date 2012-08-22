/*
 * can_channel : driver interface for managing CAN bus communication (wraps SocketCAN)
 * author: Mike Hopkins
 */

#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <math.h>
#include "can_channel.h"

can_channel::can_channel()
{
  m_fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
}

can_channel::can_channel(const char *interface_name)
{
  m_fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  open_channel(interface_name);
}

int can_channel::get_fd()
{
  /* get file descriptor for can interface */
  return m_fd;
}

int can_channel::open_channel(const char *interface_name)
{
  /* open can interface */  
  struct ifreq ifr;
  struct sockaddr_can addr;
  ifr.ifr_ifindex = 0;
  if (strcmp(interface_name, "*") != 0)
  {
    strcpy(ifr.ifr_name, interface_name);
    ioctl(m_fd, SIOCGIFINDEX, &ifr);
  }
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  return bind(m_fd, (struct sockaddr *)&addr, sizeof(addr));
}

int can_channel::set_loopback(int enable)
{
  /* set loopback option for SocketCAN */
  return setsockopt(m_fd, SOL_CAN_RAW, CAN_RAW_LOOPBACK,
    &enable, sizeof(enable));
}

int can_channel::get_loopback()
{
  /* get loopback option for SocketCAN */
  socklen_t len;
  int enable = 0;
  getsockopt(m_fd, SOL_CAN_RAW, CAN_RAW_LOOPBACK,
    &enable, &len);
  return enable;
}

int can_channel::set_timeout(double timeout)
{
  /* set receive timeout in seconds */
  int flags;
  int nonblock;
  struct timeval tv;
  if (timeout < 0)
  {
    nonblock = 0;
    tv.tv_sec = 0; 
    tv.tv_usec = 0; 
  }
  else if (timeout == 0)
  {
    nonblock = 1;
    tv.tv_sec = 0; 
    tv.tv_usec = 0; 
  }
  else
  {
    nonblock = 0;
    tv.tv_sec = floor(timeout);
    tv.tv_usec = floor((timeout - tv.tv_sec) * 1e6);
  }
  flags = fcntl(m_fd, F_GETFL, 0);
  if (nonblock)
    fcntl(m_fd, F_SETFL, flags | O_NONBLOCK);
  else
    fcntl(m_fd, F_SETFL, flags & ~O_NONBLOCK);
  return setsockopt(m_fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
}

double can_channel::get_timeout()
{
  /* get receive timeout in seconds */
  int flags;
  struct timeval tv;
  double timeout = 0;
  socklen_t len = sizeof(tv);
  getsockopt(m_fd, SOL_SOCKET, SO_RCVTIMEO, &tv, &len);
  timeout = tv.tv_sec + tv.tv_usec*1e-6;
  flags = fcntl(m_fd, F_GETFL, 0);
  if (flags & O_NONBLOCK)
    return 0;
  else if (timeout == 0)
    return -1;
  return timeout;
}

int can_channel::send(struct can_frame *frame)
{
  /* send can_frame */
  return write(m_fd, frame, sizeof(*frame));
}

int can_channel::receive(struct can_frame *frame)
{
  /* receive can_frame */
  return read(m_fd, frame, sizeof(*frame));
}

void can_channel::flush()
{
  /* flush receive buffer */
  double timeout = get_timeout();
  struct can_frame frame;
  set_timeout(0);
  while (read(m_fd, &frame, sizeof(frame)) > 0) {};
  set_timeout(timeout);
}

void can_channel::close_channel()
{
  /* close can interface */
  if (m_fd > 0) 
    close(m_fd);
  m_fd = -1;
}

can_channel::~can_channel()
{
  if (m_fd > 0) 
    close(m_fd);
  m_fd = -1;
}
