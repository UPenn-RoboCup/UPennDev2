// use the newer alsa api
#define ALSA_PCM_NEW_HW_PARAMS_API
#include <alsa/asoundlib.h>

#include <stdio.h>

#include "sound_params.h"
#include "alsa_util.h"
#include "dtmf.h"


// transmitter and reciever handles
snd_pcm_t *tx;
snd_pcm_t *rx;
// transmitter and reciever parameter objects
snd_pcm_hw_params_t *txParams;
snd_pcm_hw_params_t *rxParams;

// current audio frame number
static long frameNumber = 0;

// receive buffer
short rxBuffer[PSAMPLE];
// number of frames in buffer
int nrxFrames = 0;
// current buffer index
int irxBuffer = 0;

int open_transmitter() {
  // open transmitter (speakers)
  int ret = snd_pcm_open(&tx, "default", SND_PCM_STREAM_PLAYBACK, 0);
  if (ret < 0) {
    fprintf(stderr, "unable to open transmitter pcm device: %s\n", snd_strerror(ret));
    exit(1);
  }

  return 0;
}

int open_receiver() {
  // open receiver (microphones)
  int ret = snd_pcm_open(&rx, "default", SND_PCM_STREAM_CAPTURE, 0);
  if (ret < 0) {
    fprintf(stderr, "unable to open receiver pcm device: %s\n", snd_strerror(ret));
    exit(1);
  }

  return 0;
}

int init_devices() {
  print_alsa_lib_version();

  // open devices
  open_transmitter();
  open_receiver();

  // allocate parameter objects
  printf("opening transmitter audio device..."); fflush(stdout);
  snd_pcm_hw_params_alloca(&txParams);
  printf("done\n");
  printf("opening reciever audio device..."); fflush(stdout);
  snd_pcm_hw_params_alloca(&rxParams);
  printf("done\n");

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

  int rc;
  unsigned int val;
  int dir;
  snd_pcm_hw_params_get_period_time(rxParams, &val, &dir);
  long loops = 5000000 / val;

  snd_pcm_uframes_t frames = NFRAMES;

  nrxFrames = 0;
  irxBuffer = 0;
  while (1) {

    if (nrxFrames + frames < PFRAMES) {
      // all frames fit within buffer, read all available frames
      int rframes = snd_pcm_readi(rx, rxBuffer+irxBuffer, frames);
      if (rframes == -EPIPE) {
        // EPIPE mean overrun
        fprintf(stderr, "overrun occurred\n");
        snd_pcm_prepare(rx);
        // reset rx buffer
        irxBuffer = 0;
        continue;
      } else if (rframes < 0) {
        fprintf(stderr, "error from read: %s\n", snd_strerror(rc));
        // reset rx buffer
        irxBuffer = 0;
        continue;
      } else if (rframes != (int)frames) {
        fprintf(stderr, "short read: read %d frames\n", rc);
        // reset rx buffer
        irxBuffer = 0;
        continue;
      } 
      
      // update rx buffer
      nrxFrames += rframes;
      irxBuffer += rframes * SAMPLES_PER_FRAME;

    } else {
      // read enough frames to fill buffer
      int nframes = PFRAMES - nrxFrames;
      int rframes = snd_pcm_readi(rx, rxBuffer+irxBuffer, nframes);
      if (rframes == -EPIPE) {
        // EPIPE mean overrun
        fprintf(stderr, "overrun occurred\n");
        snd_pcm_prepare(rx);
        // reset rx buffer
        irxBuffer = 0;
        continue;
      } else if (rframes < 0) {
        fprintf(stderr, "error from read: %s\n", snd_strerror(rc));
        // reset rx buffer
        irxBuffer = 0;
        continue;
      } else if (rframes != (int)nframes) {
        fprintf(stderr, "short read: read %d frames\n", rc);
        // reset rx buffer
        irxBuffer = 0;
        continue;
      } 
      // process audio sample
      check_tone(rxBuffer);

      // read remaining audio from buffer
      nframes = NFRAMES - rframes;
      rframes = snd_pcm_readi(rx, rxBuffer, nframes);
      if (rframes == -EPIPE) {
        // EPIPE mean overrun
        fprintf(stderr, "overrun occurred\n");
        snd_pcm_prepare(rx);
        // reset rx buffer
        irxBuffer = 0;
        continue;
        
      } else if (rframes < 0) {
        fprintf(stderr, "error from read: %s\n", snd_strerror(rc));
        // reset rx buffer
        irxBuffer = 0;
        continue;

      } else if (rframes != (int)nframes) {
        fprintf(stderr, "short read: read %d frames\n", rc);
        // reset rx buffer
        irxBuffer = 0;
        continue;

      } 

      // update rx buffer
      nrxFrames = rframes;
      irxBuffer = rframes * SAMPLES_PER_FRAME;
    }
  }

  snd_pcm_drain(tx);
  snd_pcm_drain(rx);

  printf("closing transmitter device..."); fflush(stdout);
  snd_pcm_close(rx);
  printf("done\n");
  printf("closing receiver device..."); fflush(stdout);
  snd_pcm_close(tx);
  printf("done\n");

  return 0;
}


