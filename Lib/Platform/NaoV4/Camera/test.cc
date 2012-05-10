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


#define NUM_FRAME_BUFFERS 4

typedef unsigned char uint8;
typedef unsigned int uint32;

struct v4l2_buffer *buffers;
uint32 *imBuffer[NUM_FRAME_BUFFERS];

char cameraDevice[] = "/dev/video0";
int cameraFd = 0;


int set_camera_param(const char *key, int value) {
  // attempt to set camera parameter in a loop
  int maxTries = 1000;
  int usleepInt = 10000;
  struct v4l2_control control;
}

void queryCameraParams() {
  struct v4l2_queryctrl ctrl;
  for (int i = V4L2_CID_BASE; i <= V4L2_CID_LASTP1+1000000; i++) {
    ctrl.id = i;
    if (ioctl(cameraFd, VIDIOC_QUERYCTRL, &ctrl) == -1) {
      continue;
    }
    printf("  %d %s %d %d %d\n", i, ctrl.name, ctrl.minimum, ctrl.maximum, ctrl.default_value);
  }

//  LOG_INFO("Brightness: %d\nContrast: %d\nSaturation: %d\nHue: %d\nAuto White: %d\nWhite balance: %d\nExposure: %d\nGain: %d\nHFlip: %d\nVFlip: %d\nSharpness: %d\nBacklight: %d",
//         V4L2_CID_BRIGHTNESS,
//         V4L2_CID_CONTRAST,
//         V4L2_CID_SATURATION,
//         V4L2_CID_HUE,
//         V4L2_CID_AUTO_WHITE_BALANCE,
//         V4L2_CID_DO_WHITE_BALANCE,
//         V4L2_CID_EXPOSURE,
//         V4L2_CID_GAIN,
//         V4L2_CID_HFLIP,
//         V4L2_CID_VFLIP,
//         V4L2_CID_SHARPNESS,
//         V4L2_CID_BACKLIGHT_COMPENSATION);
}


int main() {
  struct v4l2_format format;
  struct v4l2_streamparm streamparm;
  struct v4l2_requestbuffers reqbuf;
  struct v4l2_buffer buffer;
  int i;

  printf("opening video device..."); fflush(stdout);
  cameraFd = open(cameraDevice, O_RDWR);
  if (cameraFd == 0) {
    printf("failed to open video device\n");
    return -1;
  }
  printf("done\n");


  printf("setting image resolution..."); fflush(stdout);
  memset(&format, 0, sizeof(struct v4l2_format));
  format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (ioctl(cameraFd, VIDIOC_G_FMT, &format) < 0) {
    printf("failed to get format: %s\n", strerror(errno));
    return -1;
  }

  format.fmt.pix.width = 640;
  format.fmt.pix.height = 480;
  format.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
  format.fmt.pix.field = V4L2_FIELD_NONE;
  if (ioctl(cameraFd, VIDIOC_S_FMT, &format) < 0) {
    printf("failed to set format\n");
    return -1;
  }

  if (format.fmt.pix.width != 640 || format.fmt.pix.height != 480) {
    printf("set image size does not match\n");
    return -1;
  }
  printf("done\n");

  printf("setting frame rate..."); fflush(stdout); 
  memset(&streamparm, 0, sizeof(struct v4l2_streamparm));
  streamparm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (ioctl(cameraFd, VIDIOC_G_PARM, &streamparm) != 0) {
    printf("failed to get stream parameters\n");
    return -1;
  }

  // bug in camera driver, in order to get 30fps set to default 
  //    frame rate (1/0) not 1/30
  streamparm.parm.capture.timeperframe.numerator = 1;
  streamparm.parm.capture.timeperframe.denominator = 0;
  if (ioctl(cameraFd, VIDIOC_S_PARM, &streamparm) != 0) {
    printf("failed to set frame rate\n");
    return -1;
  }
  printf("done\n");

  printf("setting up buffers..."); fflush(stdout);
  memset(&reqbuf, 0, sizeof(reqbuf));
  reqbuf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  reqbuf.memory = V4L2_MEMORY_MMAP;
  reqbuf.count = NUM_FRAME_BUFFERS;
  if (ioctl(cameraFd, VIDIOC_REQBUFS, &reqbuf) < 0) {
    printf("failed to set buffer request mode");
    return -1;
  }
  int numBuffers = reqbuf.count;
  printf("%d buffers...done\n", numBuffers);

  printf("mapping buffers..."); fflush(stdout);
  buffers = (struct v4l2_buffer *)calloc(numBuffers, sizeof(struct v4l2_buffer));
  if (buffers == NULL) {
    printf("failed to allocate memory for buffers\n");
    return -1;
  }

  for (int i = 0; i < numBuffers; i++) {
    memset(&buffer, 0, sizeof(buffer));
    buffer.type = reqbuf.type;
    buffer.memory = V4L2_MEMORY_MMAP;
    buffer.index = i;
    if (ioctl(cameraFd, VIDIOC_QUERYBUF, &buffer) < 0) {
      printf("error querying buffer\n");
      return -1;
    }
    buffers[i].length = buffer.length;
    imBuffer[i] = (uint32 *)mmap(NULL, buffer.length,   
                                  PROT_READ | PROT_WRITE, 
                                  MAP_SHARED, 
                                  cameraFd, 
                                  buffer.m.offset);

    if (MAP_FAILED == imBuffer[i]) {
      printf("mmap error mapping buffer\n");
      // free currently mapped buffers
      for (int j = 0; j < i; j++) {
        munmap(imBuffer[j], buffers[j].length);
      }
      return -1;
    }
  }
  printf("done\n");


  printf("querying camera params:\n");
  queryCameraParams();

  printf("starting camera stream..."); fflush(stdout);
  i = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (ioctl(cameraFd, VIDIOC_STREAMON, &i) < 0) {
    printf("failed to begin streaming\n");
    return -1;
  }
  printf("done\n");

}


