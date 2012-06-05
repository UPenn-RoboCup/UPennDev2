#ifndef __SOUND_COMM_THREAD_H__
#define __SOUND_COMM_THREAD_H__

// use the newer alsa api
#define ALSA_PCM_NEW_HW_PARAMS_API
#include <alsa/asoundlib.h>

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>
#include <fcntl.h>
#include <pthread.h>
#include <sys/time.h>
#include <sys/types.h>

#include <queue>
#include <vector>

#include "sound_params.h"
#include "alsa_util.h"
#include "dtmf.h"

struct DetStruct {
  int count;
  double time;
  int lIndex;
  int rIndex;
  char symbol;
};

int open_transmitter();

int open_receiver();

int init_devices();

void *sound_comm_rx_thread_func(void*);

void sound_comm_rx_thread_cleanup();

void sound_comm_thread_queue_pcm(std::vector<short> *pcm);

int sound_comm_thread_init();

DetStruct sound_comm_thread_get_detection();

#endif
