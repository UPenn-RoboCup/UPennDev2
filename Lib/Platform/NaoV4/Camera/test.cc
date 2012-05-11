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

#include "cam_util.h"
#include "timeScalar.h"

#define WIDTH 640
#define HEIGHT 480

#define NUM_FRAME_BUFFERS 4

#define NCAMERA_DEVICES 2

// camera 
const char *cameraDevices[] = {"/dev/video0", "/dev/video1"};
static int cameraFd[NCAMERA_DEVICES];
static int nbuf[NCAMERA_DEVICES];
struct v4l2_buffer *v4l2buffers[NCAMERA_DEVICES];
uint32 *imbuffers[NCAMERA_DEVICES];
int enqueued[NCAMERA_DEVICES];





int main() {
  // initialize each camera
  for (int i = 0; i < NCAMERA_DEVICES; i++) {
    cameraFd[i] = init_camera(cameraDevices[i], WIDTH, HEIGHT);
    if (cameraFd[i] < 0) {
      printf("unable to init camera: %s\n", cameraDevices[i]);
      return -1;
    }
    
    nbuf[i] = init_mmap(cameraFd[i], &v4l2buffers[i], &imbuffers[i], NUM_FRAME_BUFFERS);
    if (nbuf[i] < 0) {
      printf("unable to init memory map: %s\n", cameraDevices[i]);
      return -1;
    }

    if (start_stream(cameraFd[i]) < 0) {
      printf("unable to start camera stream: %s\n", cameraDevices[i]);
      return -1;
    } 
  }

  int nframe = 0;
  struct v4l2_buffer *currV4l2Buf;
  struct v4l2_buffer *nextV4l2Buf;

  currV4l2Buf = (struct v4l2_buffer *)malloc(sizeof(struct v4l2_buffer));
  if (currV4l2Buf == NULL) {
    printf("unable to allocate current buffer memory.\n");
    return -1;
  }
  nextV4l2Buf = (struct v4l2_buffer *)malloc(sizeof(struct v4l2_buffer));
  if (nextV4l2Buf == NULL) {
    printf("unable to allocate next buffer memory.\n");
    return -1;
  }

  int ind = 0;
  int ibuf = 0;
  printf("starting camera loop...\n");
  double t0 = time_scalar();
  while (1) {
    int ret = grab_frame(cameraFd[ind],
                          v4l2buffers[ind],
                          currV4l2Buf,
                          nextV4l2Buf,
                          nbuf[ind],
                          enqueued[ind],
                          ibuf,
                          nframe);
    
    if (ret < 0) {
      printf("failed to grab frame\n");
    }
    if (nframe % 100 == 0) {
      printf("fps: %f\n", (100 / (time_scalar() - t0)));
      t0 = time_scalar();
    }
    usleep(1000);
  }
    
}


