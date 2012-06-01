#ifndef __SOUND_COMM_THREAD_H__
#define __SOUND_COMM_THREAD_H__

// use the newer alsa api
#define ALSA_PCM_NEW_HW_PARAMS_API
#include <alsa/asoundlib.h>

#include <math.h>
#include <stdio.h>
#include <signal.h>
#include <unistd.h>
#include <fcntl.h>
#include <pthread.h>

#include "sound_params.h"
#include "alsa_util.h"
#include "dtmf.h"

int open_transmitter();

int open_receiver();

int init_devices();

void *sound_comm_rx_thread_func(void*);

void sound_comm_rx_thread_cleanup();

#endif
