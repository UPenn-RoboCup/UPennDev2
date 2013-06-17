
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "hokuyo.h"

speed_t BaudToSpeed(int baud) {
  switch (baud) 
  {
    case 2400:
      return B2400;
    case 4800:
      return B4800;
    case 9600:
      return B9600;
    case 19200:
      return B19200;
    case 38400:
      return B38400;
    case 57600:
      return B57600;
    case 115200:
      return B115200;
    case 230400:
      return B230400;
    case 460800:
#ifndef __APPLE__
      return B460800;
#endif
    case 921600:
#ifndef __APPLE__
      return B921600;
#endif
    case 1000000:
#ifndef __APPLE__
      return B1000000;
#endif
    case 2000000:
#ifndef __APPLE__
      return B2000000;
#endif
    default:
      fprintf(stderr, "unknown baud rate\n");
      return B0;
  }
}

int hokuyo_open(struct_hokuyo *h_dev) {
  if ((h_dev->fd = open(h_dev->device, O_RDWR | O_NOCTTY )) < 0) {
    fprintf(stderr, "Unable to open device %s\n", h_dev->device);
    return -1;
  }

  if (tcgetattr(h_dev->fd, &h_dev->oldterm) < 0) {
    fprintf(stderr, "Unable to get old serial port attributes\n");
    return -1;
  }

  /* Set baudrate */
  int baudrate = 115200;
  speed_t baud_speed = BaudToSpeed(baudrate);
  /* Get current port setting */
  if (tcgetattr(h_dev->fd, &h_dev->newterm) < 0) {
    fprintf(stderr, "Unable to get serial port attributes\n");
    return -1;
  }
  /* initialize the port to standard configuration */
  cfmakeraw(&h_dev->newterm);
  /* Set input baudrate */
  if (cfsetispeed(&h_dev->newterm, baud_speed) < 0) {
    fprintf(stderr, "Unable to set input baud rate\n");
    return -1;
  }
  /* Set output baudrate */
  if (cfsetospeed(&h_dev->newterm, baud_speed) < 0) {
    fprintf(stderr, "Unable to set output baud rate\n");
    return -1;
  }
  /* Set new attributes */
  if (tcsetattr(h_dev->fd, TCSAFLUSH, &h_dev->newterm) < 0) {
    fprintf(stderr, "Unable to set new attributes\n");
  }
  /* clean queue */
  tcflush(h_dev->fd, TCIOFLUSH);

  /* set IO_BLOCKING_W_TIMEOUT */
  int flags = fcntl(h_dev->fd, F_GETFL);
  fcntl(h_dev->fd, F_SETFL, flags & (~O_NONBLOCK));

  return 0;
}

int hokuyo_close(struct_hokuyo *h_dev) {
  /* Restore old ternimal setting */
  if (tcsetattr(h_dev->fd, TCSANOW, &h_dev->oldterm) < 0) {
    fprintf(stderr, "Unable to restore attributes\n");
    return -1;
  }

  if (close(h_dev->fd) < 0) {
    fprintf(stderr, "unable to close device\n");
    return -1;
  }
  return 0;
}

int hokuyo_stream_on(struct_hokuyo *h_dev) {
  char line[100];

  char cmd[3];
  char resp1[2];
  char resp2[2];

  /* turn on command */
  memcpy(cmd, "BM\n", 3);
  memcpy(resp1, "00", 2);
  memcpy(resp2, "02", 2);
  
  /* Set nonblocking IO*/
  int flags = fcntl(h_dev->fd, F_GETFL);
  fcntl(h_dev->fd, F_SETFL, flags & O_NONBLOCK);

  /* read all chars */
  char c[1000];
  while (read(h_dev->fd, c, 1000) > 0) {};
  printf("finish clean\n" );

  /* Set back to blocking IO */
  flags = fcntl(h_dev->fd, F_GETFL);
  fcntl(h_dev->fd, F_SETFL, flags & (~O_NONBLOCK));

  /* Write the command */
  int bytes_written_total = 0;
  int bytes_written;
  int bytes_left = 3;
  bytes_written_total = write(h_dev->fd, cmd, 3);

  tcdrain(h_dev->fd);

  return 0;
}

int hokuyo_stream_off(struct_hokuyo *h_dev) {
  char line[100];

  char cmd[3];
  char resp1[2];
  char resp2[2];

  /* turn on command */
  memcpy(cmd, "QT\n", 3);
  memcpy(resp1, "00", 2);
  memcpy(resp2, "00", 2);
  
//  /* Set nonblocking IO*/
//  int flags = fcntl(h_dev->fd, F_GETFL);
//  fcntl(h_dev->fd, F_SETFL, flags & O_NONBLOCK);
//
//  /* read all chars */
//  char c[1000];
//  while (read(h_dev->fd, c, 1000) > 0) {};
//
//  /* Set back to blocking IO */
//  flags = fcntl(h_dev->fd, F_GETFL);
//  fcntl(h_dev->fd, F_SETFL, flags & (~O_NONBLOCK));

  /* Write the command */
  int bytes_written_total = 0;
  int bytes_written;
  int bytes_left = 3;
  bytes_written_total = write(h_dev->fd, cmd, 3);

  tcdrain(h_dev->fd);


  return 0;
}

int hokuyo_read(struct_hokuyo *h_dev) {
  char seq[2] = {0x0A, 0x0A}; // _termSequence
//  _numTerChars = 2;
//  _retTermSequence = retTermSequence;

  // Set_IO_BLOCK_W_TIMEOUT_W_TERM_SEQUENCE seq, 2, true
  
#define max_len 15000
  int timeout_us = 500000;
  fd_set watched_fds;
  struct timeval timeout, start, end;
  FD_ZERO(&watched_fds);
  FD_SET(h_dev->fd, &watched_fds);
  timeout.tv_sec = timeout_us / 1000000;
  timeout.tv_usec = timeout_us % 1000000;
  int bytes_read_total = 0;
//  int bytes_left = 3372;
  int bytes_left = max_len;
  int retval;
  int bytes_read;
  int charsMatched = 0;
  char packet[max_len];

//  while (bytes_left) {
//    if ((retval = select(h_dev->fd + 1, &watched_fds, NULL, NULL, &timeout)) < 1) {
//      return -1;
//    }
//    bytes_read = read(h_dev->fd, &(packet[bytes_read_total]), bytes_left);
//    if (bytes_read > 0) {
//      bytes_read_total += bytes_read;
//      bytes_left       -= bytes_read;
//    }
//  }
//  return bytes_read_total;

  while (bytes_left) {
    if ((retval = select(h_dev->fd + 1, &watched_fds, NULL, NULL, &timeout)) < 1) {
      return -1;
    }
    bytes_read = read(h_dev->fd, &(packet[bytes_read_total]), 1);
    if (bytes_read == 1) {
      if (packet[bytes_read_total] == seq[charsMatched]) {
        charsMatched++;
      } else {
        charsMatched = 0;
      }
      bytes_read_total += bytes_read;
      bytes_left -= bytes_read;
    }
    if (charsMatched == 2) {
      int i = 0;
      for (i = 0; i < 20; i++) 
        printf("%d ", packet[bytes_read_total]);
      printf("\n");
      return bytes_read_total;
    }
  }
  int ret = read(h_dev->fd, &packet, max_len);
  return 1;
}
