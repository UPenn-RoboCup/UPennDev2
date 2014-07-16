/*
  Author: Stephen McGill 10/10
*/

#include <lua.h>
#include <lualib.h>
#include <lauxlib.h>
#include <libuvc/libuvc.h>

#include <string.h>

#include <stdlib.h>
#include <stdint.h>

// for usleep
#include <unistd.h>

/* metatable name for uvc */
#define MT_NAME "uvc2_mt"

#define DEBUG

typedef struct uvc2_dev {
  uvc_context_t *ctx;
  uvc_device_t *dev;
  uvc_device_handle_t *devh;
  uvc_stream_ctrl_t ctrl;
  uint8_t init;
  double t_frame;
  uint32_t count;
  void * data;
  size_t data_bytes;
} uvc2_dev;

static uvc2_dev * lua_checkuvc2(lua_State *L, int narg) {
  void *ud = luaL_checkudata(L, narg, MT_NAME);
  luaL_argcheck(L, ud != NULL, narg, "invalid uvc userdata");
  return (uvc2_dev *) ud;
}

static int lua_uvc2_index(lua_State *L) {
  if (!lua_getmetatable(L, 1)) {
    /* push metatable */
    lua_pop(L, 1); 
    return 0;
  }
  lua_pushvalue(L, 2); /* copy key */
  lua_rawget(L, -2); /* get metatable function */
  lua_remove(L, -2);  /* delete metatable */
  return 1;
}

void cb(uvc_frame_t *frame, void *ptr) {
  uvc2_dev *ud = (uvc2_dev *) ptr;
  struct timeval t;
  gettimeofday(&t, NULL);
  
#ifdef DEBUG
  printf("Running CB\n");
#endif
  
  size_t nbytes = frame->data_bytes;
  if(!ud->data){
    ud->data = malloc(nbytes);
  }
  memcpy(ud->data, frame->data, nbytes);
  ud->data_bytes = nbytes;
  ud->count++;
  ud->t_frame = t.tv_sec + 1E-6*t.tv_usec;
}

static int lua_uvc2_get_frame(lua_State *L) {
  uvc2_dev *ud = lua_checkuvc2(L, 1);
  lua_pushlightuserdata(L, ud->data);
  lua_pushnumber(L, ud->data_bytes);
  lua_pushnumber(L, ud->count);  
  lua_pushnumber(L, ud->t_frame);
  return 4;
}

static int lua_uvc2_stream_on(lua_State *L) {
  uvc2_dev *ud = lua_checkuvc2(L, 1);
  
  if(!ud->init){
    return 0;
  }
  
  /* Start the video stream. The library will call user function cb:
   *   cb(frame, (void*) 12345)
   */
//  uvc_context_t *ctx = ud->ctx;
//  uvc_device_t *dev = ud->dev;
  uvc_device_handle_t *devh = ud->devh;
  uvc_stream_ctrl_t* ctrl = &(ud->ctrl);
  uvc_error_t res;
  
#ifdef DEBUG
  printf("STREAM start");
#endif
  res = uvc_start_streaming(devh, ctrl, cb, ud, 0);
  if (res < 0) {
    /* unable to start stream */
    uvc_perror(res, "start_streaming");
    return luaL_error(L, "Bad stream start");
  }
  return 0;
}

static int lua_uvc2_stream_off(lua_State *L) {
  uvc2_dev *ud = lua_checkuvc2(L, 1);
  
  if(!ud->init){
    return 0;
  }
  
#ifdef DEBUG
    printf("uvc_stop_streaming!\n");
#endif
  
  uvc_device_handle_t *devh = ud->devh;
  
  /* End the stream. Blocks until last callback is serviced */
  uvc_stop_streaming(devh);

  return 0;
}

/* Just make a big table */
static int lua_uvc2_get_param(lua_State *L) {
  uvc2_dev *ud = lua_checkuvc2(L, 1);
  uvc_device_handle_t *devh = ud->devh;
  uint8_t mode8;
  uint16_t mode16;
  int16_t mode16s;
  uint32_t mode32;
  uvc_error_t res;
  
  res = uvc_get_ae_mode(devh, &mode8, UVC_GET_CUR);
  if (res < 0) { uvc_perror(res, "uvc_get_ae_mode!!!"); } else
  printf("uvc_ae mode %d\n", mode8);
  
  res = uvc_get_ae_priority(devh, &mode8, UVC_GET_CUR);
  if (res < 0) { uvc_perror(res, "uvc_get_ae_priority!!!"); } else
  printf("uvc_get_ae_priority %d\n", mode8);
  
  res = uvc_get_exposure_abs(devh, &mode32, UVC_GET_CUR);
  if (res < 0) { uvc_perror(res, "uvc exposure!!!"); } else
  printf("uvc_get_exposure_abs %u\n", mode32);
  
  res = uvc_get_focus_auto(devh, &mode8, UVC_GET_CUR);
  if (res < 0) { uvc_perror(res, "uvc_get_focus_auto!!!"); } else
  printf("uvc_get_focus_auto %d\n", mode8);
  
  res = uvc_get_focus_abs(devh, &mode16, UVC_GET_CUR);
  if (res < 0) { uvc_perror(res, "uvc_get_focus_abs!!!"); } else
  printf("uvc_get_focus_abs %u\n", mode16);

  res = uvc_get_brightness(devh, &mode16s, UVC_GET_DEF);
  if (res < 0) { uvc_perror(res, "uvc brightness!!!"); } else
  printf("uvc_get_brightness %d\n", mode16s);
  
  res = uvc_get_sharpness(devh, &mode16, UVC_GET_CUR);
  if (res < 0) { uvc_perror(res, "uvc_get_sharpness!!!"); } else
  printf("uvc_get_sharpness %d\n", mode16);
  
  res = uvc_get_contrast(devh, &mode16, UVC_GET_CUR);
  if (res < 0) { uvc_perror(res, "uvc contrast!!!"); } else
  printf("uvc_get_contrast %u\n", mode16);
  
  // Problems
  res = uvc_get_saturation(devh, &mode16, UVC_GET_CUR);
  if (res < 0) { uvc_perror(res, "uvc_get_saturation!!!"); } else
  printf("uvc_get_saturation %d\n", mode16);
  
  res = uvc_get_backlight_compensation(devh, &mode16, UVC_GET_CUR);
  if (res < 0) { uvc_perror(res, "uvc_get_backlight_compensation!!!"); } else
  printf("uvc_get_backlight_compensation %u\n", mode16);
  
  res = uvc_get_gain(devh, &mode16, UVC_GET_CUR);
  if (res < 0) { uvc_perror(res, "uvc_get_gain!!!"); } else
  printf("uvc_get_gain %d\n", mode16);
  
  res = uvc_get_white_balance_temperature_auto(devh, &mode8, UVC_GET_CUR);
  if (res < 0) { uvc_perror(res, "uvc_get_white_balance_temperature_auto!!!"); } else
  printf("uvc_get_white_balance_temperature_auto %d\n", mode8);
  
  res = uvc_get_white_balance_temperature(devh, &mode16, UVC_GET_CUR);
  if (res < 0) { uvc_perror(res, "uvc_get_white_balance_temperature!!!"); } else
  printf("uvc_get_white_balance_temperature %d\n", mode16);
  
  return 0;
}

/* Take in a big table */
static int lua_uvc2_set_param(lua_State *L) {
  uvc2_dev *ud = lua_checkuvc2(L, 1);
  uvc_device_handle_t *devh = ud->devh;
  //uint8_t val = luaL_checknumber(L, 2);
  /* e.g., turn off auto exposure */
  uvc_error_t res;
  
  uint8_t m = 1;
  res = uvc_set_ae_mode(devh, m);
  if (res < 0) { uvc_perror(res, "uvc_ae mode"); }
  
  // No effect in manual exposure
  /*
  uint8_t p = 0;
  res = uvc_set_ae_priority(devh, p);
  if (res < 0) { uvc_perror(res, "uvc_ae priority"); }
  */

  // units of 0.0001 seconds
  uint32_t e = 66;
  res = uvc_set_exposure_abs(devh, e);
  if (res < 0) { uvc_perror(res, "uvc exposure"); }
  
  uint8_t af = 0;
  res = uvc_set_focus_auto(devh, af);
  if (res < 0) { uvc_perror(res, "uvc focus auto"); }
  
  uint16_t f = 0;
  res = uvc_set_focus_abs(devh, f);
  if (res < 0) { uvc_perror(res, "uvc focus abs"); }
  
  // Problems
  uint16_t sh = 128;
  res = uvc_set_sharpness(devh, sh);
  if (res < 0) { uvc_perror(res, "uvc sharpness"); }
  
  uint16_t c = 100; // 1 works
  res = uvc_set_contrast(devh, c);
  if (res < 0) { uvc_perror(res, "uvc contrast"); }
  
  int16_t b = 1; // 1 works
  res = uvc_set_brightness(devh, b);
  if (res < 0) { uvc_perror(res, "uvc brightness"); }
  
  uint16_t g = 0; // 66 works
  res = uvc_set_gain(devh, g);
  if (res < 0) { uvc_perror(res, "uvc gain"); }
  
  uint16_t s = 150;
  res = uvc_set_saturation(devh, s);
  if (res < 0) { uvc_perror(res, "uvc_set_saturation!!!"); }
  
  res = uvc_set_white_balance_temperature_auto(devh, 1);
  if (res < 0) { uvc_perror(res, "uvc_set_white_balance_temperature_auto!!!"); }
  
  res = uvc_set_white_balance_temperature(devh, 3300);
  if (res < 0) { uvc_perror(res, "uvc_set_white_balance_temperature!!!"); }
  
  return 0;
}

static int lua_uvc2_close(lua_State *L) {
  
  uvc2_dev *ud = lua_checkuvc2(L, 1);
  
  if(!ud->init){
#ifdef DEBUG
    printf("Not initialized!\n");
#endif
    return 0;
  }
  
  uvc_context_t *ctx = ud->ctx;
  uvc_device_handle_t *devh = ud->devh;
  uvc_device_t *dev = ud->dev;
  
  /* End the stream. Blocks until last callback is serviced */
#ifdef DEBUG
  printf("Stopping stream...\n");
  fflush(stdout);
#endif
  uvc_stop_streaming(devh);

  /* Release our handle on the device */
#ifdef DEBUG
  printf("Release handle...\n");
  fflush(stdout);
#endif
  uvc_close(devh);

  /* Release the device descriptor */
#ifdef DEBUG
  printf("Unref handle...\n");
  fflush(stdout);
#endif
  uvc_unref_device(dev);

  /* Close the UVC context. This closes and cleans up any existing device handles,
   * and it closes the libusb context if one was not provided. */
#ifdef DEBUG
  printf("Exit context...\n");
  fflush(stdout);
#endif
  uvc_exit(ctx);
  
  /* free our buffer */
  if(ud->data){
    free(ud->data);
  }
  
  ud->init = 0;
    
  return 0;
}

static int lua_uvc2_init(lua_State *L) {
  
  uvc_context_t *ctx;
  uvc_device_t *dev;
  uvc_device_handle_t *devh;
  
  uvc_error_t res;
  res = uvc_init(&ctx, NULL);
  if (res < 0) {
    uvc_perror(res, "uvc_init");
    return luaL_error(L, "UVC init");
  }
  /* Locates the first attached UVC device, stores in dev */
  /* filter devices: vendor_id, product_id, "serial_num" */
  res = uvc_find_device(ctx, &dev, 0, 0, NULL);
  if (res < 0) {
    uvc_perror(res, "uvc_find_device"); /* no devices found */
    return luaL_error(L, "UVC find");
  }
  /* Try to open the device: requires exclusive access */
  res = uvc_open(dev, &devh);
  if (res < 0) {
    /* unable to open device */
    uvc_perror(res, "uvc_open");
    return luaL_error(L, "UVC open");
  }
  /* Print out a message containing all the information that libuvc
   * knows about the device */
#ifdef DEBUG
  uvc_print_diag(devh, stderr);
#endif
  
  uvc2_dev *ud = (uvc2_dev *)lua_newuserdata(L, sizeof(uvc2_dev));
  luaL_getmetatable(L, MT_NAME);
  lua_setmetatable(L, 1);
  ud->ctx = ctx;
  ud->dev = dev;
  ud->devh = devh;
  ud->data = NULL;
  ud->data_bytes = 0;
  ud->count = 0;
  ud->t_frame = 0;
  uvc_stream_ctrl_t* ctrl = &(ud->ctrl);
  
  /* Try to negotiate a 640x480 30 fps YUYV stream profile */
  res = uvc_get_stream_ctrl_format_size(
      devh, ctrl, /* result stored in ctrl */
      UVC_FRAME_FORMAT_YUYV, /* YUV 422, aka YUV 4:2:2. try _COMPRESSED */
      640, 480, 30 /* width, height, fps */
  );
  /* Print out the result */
  uvc_print_stream_ctrl(ctrl, stderr);
  if (res < 0) {
    /* device doesn't provide a matching stream */
    uvc_perror(res, "get_mode");
    return luaL_error(L, "Bad get mode");
  }
  /*
#ifdef DEBUG
  printf("Starting the stream...\n");
#endif
  res = uvc_start_streaming(devh, ctrl, cb, ud, 0);
  if (res < 0) {
    // unable to start stream
    uvc_perror(res, "start_streaming");
    return luaL_error(L, "Bad stream start");
  }
  */
  ud->init = 1;
  
  return 1;
}

static const struct luaL_Reg uvc_methods [] = {
  {"get_frame", lua_uvc2_get_frame},
  {"stream_on", lua_uvc2_stream_on},
  {"stream_off", lua_uvc2_stream_off},
  {"get_param", lua_uvc2_get_param},
  {"set_param", lua_uvc2_set_param},
  {"close", lua_uvc2_close},
  {"__gc", lua_uvc2_close},
  {"__index", lua_uvc2_index},
  {NULL, NULL}
};

static const struct luaL_Reg uvc_functions [] = {
  {"init", lua_uvc2_init},
  {NULL, NULL}
};

#ifdef __cplusplus
extern "C"
#endif
int luaopen_uvc2 (lua_State *L) {
    /* create metatable for uvc module */
    luaL_newmetatable(L, MT_NAME);

#if LUA_VERSION_NUM == 502
    luaL_setfuncs(L, uvc_methods, 0);
	  luaL_newlib(L, uvc_functions);
#else 
    luaL_register(L, NULL, uvc_methods);
    luaL_register(L, "uvc2", uvc_functions);
#endif
    return 1;
}
