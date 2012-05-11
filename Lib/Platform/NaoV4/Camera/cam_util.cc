#include "cam_util.h"

// initializes the camera at the given dev path
//  return the file descriptor on success and -1 on error
int init_camera(const char *dev, int width, int height) {
  struct v4l2_format format;
  struct v4l2_streamparm streamparm;

  // open device
  printf("opening video device...%s...", dev); fflush(stdout);
  int fd = open(dev, O_RDWR);
  if (fd == 0) {
    printf("failed to open video device\n");
    return -1;
  }
  printf("done\n");
  
  // set image resolution
  printf("setting image resolution..."); fflush(stdout);
  memset(&format, 0, sizeof(struct v4l2_format));
  format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (ioctl(fd, VIDIOC_G_FMT, &format) < 0) {
    printf("failed to get format: %s\n", strerror(errno));
    return -1;
  }

  format.fmt.pix.width = width;
  format.fmt.pix.height = height;
  format.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
  format.fmt.pix.field = V4L2_FIELD_NONE;
  if (ioctl(fd, VIDIOC_S_FMT, &format) < 0) {
    printf("failed to set format\n");
    return -1;
  }
  printf("(%d,%d)...", format.fmt.pix.width, format.fmt.pix.height);

  if (format.fmt.pix.width != width || format.fmt.pix.height != height) {
    printf("set image size does not match\n");
    return -1;
  }
  printf("done\n");

  // set desired frame rate
  printf("setting frame rate..."); fflush(stdout); 
  memset(&streamparm, 0, sizeof(struct v4l2_streamparm));
  streamparm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (ioctl(fd, VIDIOC_G_PARM, &streamparm) != 0) {
    printf("failed to get stream parameters\n");
    return -1;
  }

  // bug in camera driver, in order to get 30fps set to default 
  //    frame rate (1/0) not 1/30
  streamparm.parm.capture.timeperframe.numerator = 1;
  streamparm.parm.capture.timeperframe.denominator = 0;
  if (ioctl(fd, VIDIOC_S_PARM, &streamparm) != 0) {
    printf("failed to set frame rate\n");
    return -1;
  }
  printf("%d/%d...", streamparm.parm.capture.timeperframe.numerator, 
                      streamparm.parm.capture.timeperframe.denominator);
  printf("done\n");

  return fd;
}


// initialize memory map for image buffer
//  return number of buffers on success and -1 on error
int init_mmap(int fd, struct v4l2_buffer **v4l2buffers, uint32 **imbuffers) {
  struct v4l2_requestbuffers reqbuf;
  struct v4l2_buffer buffer;

  // set camera to mmap mode and initialize buffers
  printf("setting up buffers..."); fflush(stdout);
  memset(&reqbuf, 0, sizeof(reqbuf));
  reqbuf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  reqbuf.memory = V4L2_MEMORY_MMAP;
  reqbuf.count = NUM_FRAME_BUFFERS;
  if (ioctl(fd, VIDIOC_REQBUFS, &reqbuf) < 0) {
    printf("failed to set buffer request mode\n");
    return -1;
  }
  int nbuf = reqbuf.count;
  printf("%d buffers set...done\n", nbuf);

  // memory map buffers
  printf("mapping buffers..."); fflush(stdout);
  *v4l2buffers = (struct v4l2_buffer *)calloc(nbuf, sizeof(struct v4l2_buffer));
  if (*v4l2buffers == NULL) {
    printf("failed to allocate memory for buffers\n");
    return -1;
  }

  imbuffers = (uint32 **)malloc(nbuf * sizeof(uint32 *));
  if (imbuffers == NULL) {
    printf("unable to allocate image buffer pointer array\n");
    return -1;
  }

  for (int i = 0; i < nbuf; i++) {
    memset(&buffer, 0, sizeof(buffer));
    buffer.type = reqbuf.type;
    buffer.memory = V4L2_MEMORY_MMAP;
    buffer.index = i;
    if (ioctl(fd, VIDIOC_QUERYBUF, &buffer) < 0) {
      printf("error querying buffer\n");
      return -1;
    }
    (*v4l2buffers)[i].length = buffer.length;
    imbuffers[i] = (uint32 *)mmap(NULL, buffer.length,   
                                        PROT_READ | PROT_WRITE, 
                                        MAP_SHARED, 
                                        fd, 
                                        buffer.m.offset);

    if (imbuffers[i] == MAP_FAILED) {
      printf("mmap error mapping buffer\n");
      // free currently mapped buffers
      for (int j = 0; j < i; j++) {
        munmap(imbuffers[j], (*v4l2buffers)[j].length);
      }
      // free image buffer pointer array
      free(imbuffers);
      return -1;
    }
  }
  printf("done\n");

  return nbuf;
}


// iterate over all possible camera parameters and print any 
//  parameters that are supported
void query_camera_params(int fd) {
  struct v4l2_queryctrl ctrl;
  for (int i = V4L2_CID_BASE; i <= V4L2_CID_LASTP1+1000000; i++) {
    ctrl.id = i;
    if (ioctl(fd, VIDIOC_QUERYCTRL, &ctrl) == -1) {
      continue;
    }
    printf("  %d %s %d %d %d\n", i, ctrl.name, ctrl.minimum, ctrl.maximum, ctrl.default_value);
  }
}

// attempt to set camera parameter in a loop
//  return 0 on success and -1 on failure
int set_camera_param(int fd, int id, int value) {
  struct v4l2_control control;

  for (int i = 0; i < 10000; i++) {
    control.id = id;
    control.value = value;
    if (ioctl(fd, VIDIOC_S_CTRL, &control) < 0) {
      printf("failed to set parameter: %d:%d\n", id, value);
      return -1;
    }
    memset(&control, 0, sizeof(v4l2_control));
    control.id = id;
    if (ioctl(fd, VIDIOC_G_CTRL, &control) < 0) {
      printf("failed to get parameter: %d\n", id);
      return -1;
    }
    if (control.value == value) {
      printf("set %d to %d\n", id, value);
      return 0;
    }

    if (i % 10 == 0) {
      printf("Attempting to set parameter %d to %d for the %dth time\n", id, value, i);
    }

    usleep(10000);
  }

  return -1;
}


// starts the actual camera stream
//  return 0 on success and -1 on failure
int start_stream(int fd) {
  printf("starting camera stream..."); fflush(stdout);
  i = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (ioctl(fd, VIDIOC_STREAMON, &i) < 0) {
    printf("failed to begin streaming\n");
    return -1;
  }
}

