// use the newer alsa api
#define ALSA_PCM_NEW_HW_PARAMS_API
#include <alsa/asoundlib.h>

#include <stdio.h>

#include "sound_params.h"
#include "alsa_util.h"
//#include "dtmf.h"


// transmitter and reciever handles
snd_pcm_t *tx;
snd_pcm_t *rx;
// transmitter and reciever parameter objects
snd_pcm_hw_params_t *txParams;
snd_pcm_hw_params_t *rxParams;

int open_transmitter() {
  // open transmitter (speakers)
  printf("opening transmitter audio device..."); fflush(stdout);
  int ret = snd_pcm_open(&tx, "default", SND_PCM_STREAM_PLAYBACK, 0);
  if (ret < 0) {
    fprintf(stderr, "unable to open transmitter pcm device: %s\n", snd_strerror(ret));
    exit(1);
  }
  printf("done\n");

  return 0;
}

int open_receiver() {
  // open receiver (microphones)
  printf("opening reciever audio device..."); fflush(stdout);
  int ret = snd_pcm_open(&rx, "default", SND_PCM_STREAM_CAPTURE, 0);
  if (ret < 0) {
    fprintf(stderr, "unable to open receiver pcm device: %s\n", snd_strerror(ret));
    exit(1);
  }
  printf("done\n");

  return 0;
}

int init_devices() {
  print_alsa_lib_version();

  // open devices
  open_transmitter();
  open_receiver();

  // allocate parameter objects
  snd_pcm_hw_params_alloca(&txParams);
  snd_pcm_hw_params_alloca(&rxParams);

  // set parameters
  printf("setting transmitter parameters..."); fflush(stdout);
  set_device_params(tx, txParams);
  printf("done\n");
  print_device_params(tx, txParams, 0); 

  printf("setting receiver parameters..."); fflush(stdout);
  set_device_params(rx, rxParams);
  printf("done\n");
  print_device_params(rx, rxParams, 0); 
}


int main() {
  // initialize audio devices (transmitter and receiver)
  init_devices();



  return 0;
}


