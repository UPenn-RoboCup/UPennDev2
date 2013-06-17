#ifndef __HOKUYO_H__
#define __HOKUYO_H__

/* for open */
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
/* for terminal attributes */
#include <termios.h>
#include <unistd.h>


typedef struct {
  int fd;
  const char *device;
  const char *serial;
  
  struct termios oldterm;
  struct termios newterm;

} struct_hokuyo;

int hokuyo_open(struct_hokuyo *h_dev);
int hokuyo_close(struct_hokuyo *h_dev);
int hokuyo_read(struct_hokuyo *h_dev);
int hokuyo_stream_on(struct_hokuyo *h_dev);
int hokuyo_stream_off(struct_hokuyo *h_dev);

#endif
