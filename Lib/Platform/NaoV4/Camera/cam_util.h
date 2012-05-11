#ifndef __CAM_UTIL_H__
#define __CAM_UTIL_H__

#include <stdio.h>
#include <errno.h>
#include <string>
#include <string.h>
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>
#include <fcntl.h>
#include <pthread.h>

#include <map>
#include <string>
#include <vector>

#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/mman.h> 

#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <linux/videodev2.h>

typedef unsigned char uint8;
typedef unsigned int uint32;


// initializes the camera at the given dev path
//  return the file descriptor on success and -1 on error
int init_camera(const char *dev, int width, int height);


// initialize memory map for image buffer
//  return number of buffers on success and -1 on error
int init_mmap(int fd, struct v4l2_buffer **v4l2buffers, uint32 **imbuffers);


// iterate over all possible camera parameters and print any 
//  parameters that are supported
void query_camera_params(int fd);


// attempt to set camera parameter in a loop
//  return 0 on success and -1 on failure
int set_camera_param(int fd, int id, int value);


// starts the actual camera stream
//  return 0 on success and -1 on failure
int start_stream(int fd);

#endif
