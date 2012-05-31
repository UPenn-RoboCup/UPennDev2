#include "alsa_util.h"

// audio parameters
#include "sound_params.h"

void print_alsa_lib_version() {
  printf("ALSA library version: %s\n", SND_LIB_VERSION_STR);
}

void print_alsa_formats() {
  int val;

  printf("\nPCM stream types:\n");
  for (val = 0; val <= SND_PCM_STREAM_LAST; val++) {
    printf("  %s\n", snd_pcm_stream_name((snd_pcm_stream_t)val));
  }

  printf("\nPCM access types:\n");
  for (val = 0; val <= SND_PCM_ACCESS_LAST; val++) {
    printf("  %s\n", snd_pcm_access_name((snd_pcm_access_t)val));
  }

  printf("\nPCM formats:\n");
  for (val = 0; val <= SND_PCM_FORMAT_LAST; val++) {
    if (snd_pcm_format_name((snd_pcm_format_t)val) != NULL) {
      printf("  %s (%s)\n", snd_pcm_format_name((snd_pcm_format_t)val),
                            snd_pcm_format_description((snd_pcm_format_t)val));
    }
  }

  printf("\nPCM subformats:\n");
  for (val = 0; val <= SND_PCM_SUBFORMAT_LAST; val++) {
    printf("  %s (%s)\n", snd_pcm_subformat_name((snd_pcm_subformat_t)val),
                          snd_pcm_subformat_description((snd_pcm_subformat_t)val));
  }

  printf("\nPCM states:\n");
  for (val = 0; val <= SND_PCM_STATE_LAST; val++) {
    printf("  %s\n", snd_pcm_state_name((snd_pcm_state_t)val));
  }
}


int set_device_params(snd_pcm_t *handle, snd_pcm_hw_params_t *params) {
  int ret;
  int dir = 1;
  snd_pcm_uframes_t frames;

  // fill it in with default values
  ret = snd_pcm_hw_params_any(handle, params);
  if (ret < 0) {
    fprintf(stderr, "unable to initialize parameter object: %s\n", snd_strerror(ret));
    return ret;
  }

  // interleaved mode
  ret = snd_pcm_hw_params_set_access(handle, params, SND_PCM_ACCESS_RW_INTERLEAVED);
  if (ret < 0) {
    fprintf(stderr, "unable to set access type: %s\n", snd_strerror(ret));
    return ret;
  }
  // signed 16-bit little-endian format
  ret = snd_pcm_hw_params_set_format(handle, params, SND_PCM_FORMAT_S16_LE);
  if (ret < 0) {
    fprintf(stderr, "unable to set sample format: %s\n", snd_strerror(ret));
    return ret;
  }
  // two channels (stereo)
  ret = snd_pcm_hw_params_set_channels(handle, params, NCHANNEL);
  if (ret < 0) {
    fprintf(stderr, "unable to set number of channels: %s\n", snd_strerror(ret));
    return ret;
  }
  // sampling rate
  //  dir - sub unit direction (not sure what that actually means
  ret = snd_pcm_hw_params_set_rate(handle, params, SAMPLING_RATE, dir);
  if (ret < 0) {
    fprintf(stderr, "unable to set sample rate: %s\n", snd_strerror(ret));
    return ret;
  }
  // TODO: is it actually possible to set the frame size on the nao?
  /*
  // set period size
  //ret = snd_pcm_hw_params_set_period_size_near(handle, params, &frames, &dir);
  ret = snd_pcm_hw_params_set_period_size(handle, params, FRAMES_PER_PERIOD, 0);
  if (ret < 0) {
    fprintf(stderr, "unable to set sample rate: %s\n", snd_strerror(ret));
    return ret;
  }
  */
  // write the parameters to the driver
  ret = snd_pcm_hw_params(handle, params);
  if (ret < 0) {
    fprintf(stderr, "unable to set hw parameters: %s\n", snd_strerror(ret));
    return ret;
  }

  return 0;
}

void print_device_params(snd_pcm_t *handle, snd_pcm_hw_params_t *params, int full) {
  unsigned int val;
  unsigned int val2;
  int dir;
  snd_pcm_uframes_t frames;
  snd_pcm_format_t fmt;

  // display information about th e PCM interface

  // handle name
  printf("PCM handle name = '%s'\n", snd_pcm_name(handle));
  // device state
  printf("PCM state = %s\n", snd_pcm_state_name(snd_pcm_state(handle)));
  // access type
  snd_pcm_hw_params_get_access(params, (snd_pcm_access_t *)&val);
  printf("access type = %s\n", snd_pcm_access_name((snd_pcm_access_t) val));
  // pcm format
  snd_pcm_hw_params_get_format(params, &fmt);
  printf("format = '%s' (%s)\n", snd_pcm_format_name((snd_pcm_format_t)fmt),
                                 snd_pcm_format_description((snd_pcm_format_t)fmt));
  // number of channels
  snd_pcm_hw_params_get_channels(params, &val);
  printf("channels = %d\n", val);
  // sampling rate
  snd_pcm_hw_params_get_rate(params, &val, &dir);
  printf("rate = %d bps\n", val);
  // period time
  snd_pcm_hw_params_get_period_time(params, &val, &dir);
  printf("period time = %d us\n", val);
  // period size
  snd_pcm_hw_params_get_period_size(params, &frames, &dir);
  printf("period size = %d frames\n", (int)frames);

  if (full) {
    // buffer time
    snd_pcm_hw_params_get_buffer_time(params, &val, &dir);
    printf("buffer time = %d us\n", val);
    // buffer size
    snd_pcm_hw_params_get_buffer_size(params, (snd_pcm_uframes_t *)&val);
    printf("buffer size = %d frames\n", val);
    // number of periods
    snd_pcm_hw_params_get_periods(params, &val, &dir);
    printf("periods per buffer = %d\n", val);
    // sampling rate
    snd_pcm_hw_params_get_rate_numden(params, &val, &val2);
    printf("exact rate = %d/%d bps\n", val, val2);
    // significant bits
    val = snd_pcm_hw_params_get_sbits(params);
    printf("significant bits = %d\n", val);
    // tick time
    snd_pcm_hw_params_get_tick_time(params, &val, &dir);
    printf("tick time = %d us\n", val);
    // is batch
    val = snd_pcm_hw_params_is_batch(params);
    printf("is batch = %d\n", val);
    // is block transfer
    val = snd_pcm_hw_params_is_block_transfer(params);
    printf("is block transfer = %d\n", val);
    // is double
    val = snd_pcm_hw_params_is_double(params);
    printf("is double = %d\n", val);
    // is half duplex
    val = snd_pcm_hw_params_is_half_duplex(params);
    printf("is half duplex = %d\n", val);
    // is joint duplex
    val = snd_pcm_hw_params_is_joint_duplex(params);
    printf("is joint duplex = %d\n", val);
    // can overrange?
    val = snd_pcm_hw_params_can_overrange(params);
    printf("can overrange = %d\n", val);
    // can mmap sample resolution
    val = snd_pcm_hw_params_can_mmap_sample_resolution(params);
    printf("can mmap sample resolution = %d\n", val);
    // can pause
    val = snd_pcm_hw_params_can_pause(params);
    printf("can pause = %d\n", val);
    // can resume
    val = snd_pcm_hw_params_can_resume(params);
    printf("can resume = %d\n", val);
    // can sync start
    val = snd_pcm_hw_params_can_sync_start(params);
    printf("can sync start = %d\n", val);
  }
}


/*
int main() {
  printf("ALSA library version: %s\n", SND_LIB_VERSION_STR);

  long loops;
  int rc;
  int size;
  unsigned int val;
  int dir;
  snd_pcm_uframes_t frames;
  char *buffer;

  snd_pcm_t *handle;
  snd_pcm_hw_params_t *params;

  // open PCM device for playback
  printf("opening audio device for playback..."); fflush(stdout);
  rc = snd_pcm_open(&handle, "default", SND_PCM_STREAM_CAPTURE, 0);
  if (rc < 0) {
    fprintf(stderr, "unable to open pcm device: %s\n", snd_strerror(rc));
    exit(1);
  }
  printf("done\n");

  // allocate parameters struct
  snd_pcm_hw_params_alloca(&params);

  // set default parameters
  set_device_params(handle, params);

  // print out audio parameters
  print_device_params(handle, params);

  // allocate buffer for 1 period
  snd_pcm_hw_params_get_period_size(params, &frames, &dir);
  // 4 bytes per frame: 2 bytes per sample, 2 channels
  size = frames * 4; 
  buffer = (char *)malloc(size);

  //loop for 5 seconds
  snd_pcm_hw_params_get_period_time(params, &val, &dir);
  // number of loops = 5 sec (in us) divided by period time
  loops = 5000000 / val;

  while (loops > 0) {
    loops -= 1;


    rc = snd_pcm_readi(handle, buffer, frames);
    if (rc == -EPIPE) {
      // EPIPE mean overrun
      fprintf(stderr, "overrun occurred\n");
      snd_pcm_prepare(handle);
    } else if (rc < 0) {
      fprintf(stderr, "error from read: %s\n", snd_strerror(rc));
    } else if (rc != (int)frames) {
      fprintf(stderr, "short read: read %d frames\n", rc);
    }

    rc = write(1, buffer, size);
    if (rc != size) {
      fprintf(stderr, "short write: wrote %d bytes\n", rc);
    }
  }

  snd_pcm_drain(handle);

  printf("closing device..."); fflush(stdout);
  snd_pcm_close(handle);
  printf("done\n");

  free(buffer);

  return 0;
}
*/
