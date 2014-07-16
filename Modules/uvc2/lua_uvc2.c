/*
  Author: Stephen McGill 10/10
*/

#include <lua.h>
#include <lualib.h>
#include <lauxlib.h>
#include <libuvc/libuvc.h>

#include <stdint.h>
#include <stdio.h>

/* metatable name for uvc */
#define MT_NAME "uvc2_mt"

typedef struct uvc2_dev {
  uvc_context_t *ctx;
  uvc_device_t *dev;
  uvc_device_handle_t *devh;
  uvc_stream_ctrl_t ctrl;
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

static inline double time_scalar() {
  static struct timeval t;
  gettimeofday(&t, NULL);
  return t.tv_sec + 1E-6*t.tv_usec;
}

void cb(uvc_frame_t *frame, void *ptr) {
  double t = time_scalar();
  printf("GOT frame @ %lf\n", t);
}

static int lua_uvc2_stream_on(lua_State *L) {
  uvc2_dev *ud = lua_checkuvc2(L, 1);
  /* Start the video stream. The library will call user function cb:
   *   cb(frame, (void*) 12345)
   */
  uvc_context_t *ctx;
  uvc_device_handle_t *devh;
  uvc_device_t *dev;
  uvc_error_t res;
  ctx = ud->ctx;
  dev = ud->dev;
  devh = ud->devh;
  uvc_stream_ctrl_t* ctrl = &(ud->ctrl);
  
  res = uvc_start_streaming(devh, ctrl, cb, NULL, 0);
  if (res < 0) {
    /* unable to start stream */
    uvc_perror(res, "start_streaming");
    return luaL_error(L, "Bad stream start");
  }
  return 0;
}

static int lua_uvc2_stream_off(lua_State *L) {
  uvc2_dev *ud = lua_checkuvc2(L, 1);
  
  uvc_device_handle_t *devh;
  devh = ud->devh;
  
  /* End the stream. Blocks until last callback is serviced */
  uvc_stop_streaming(devh);

  return 0;
}

static int lua_uvc2_set_param(lua_State *L) {
  uvc2_dev *ud = lua_checkuvc2(L, 1);
  uvc_device_handle_t *devh;
  devh = ud->devh;
  uvc_set_ae_mode(devh, 1); /* e.g., turn on auto exposure */
  return 0;
}

static int lua_uvc2_close(lua_State *L) {
  uvc2_dev *ud = lua_checkuvc2(L, 1);
  uvc_context_t *ctx;
  uvc_device_handle_t *devh;
  uvc_device_t *dev;
  ctx = ud->ctx;
  dev = ud->dev;
  devh = ud->devh;

  /* Release our handle on the device */
  uvc_close(devh);

  /* Release the device descriptor */
  uvc_unref_device(dev);

  /* Close the UVC context. This closes and cleans up any existing device handles,
   * and it closes the libusb context if one was not provided. */
  uvc_exit(ctx);
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
    return 0;
  }
  /* Locates the first attached UVC device, stores in dev */
  /* filter devices: vendor_id, product_id, "serial_num" */
  res = uvc_find_device(ctx, &dev, 0, 0, NULL);
  if (res < 0) {
    uvc_perror(res, "uvc_find_device"); /* no devices found */
    return 0;
  }
  /* Try to open the device: requires exclusive access */
  res = uvc_open(dev, &devh);
  if (res < 0) {
    /* unable to open device */
    uvc_perror(res, "uvc_open");
    return 0;
  }
  /* Print out a message containing all the information that libuvc
   * knows about the device */
  uvc_print_diag(devh, stderr);
  
  uvc2_dev *ud = (uvc2_dev *)lua_newuserdata(L, sizeof(uvc2_dev));
  luaL_getmetatable(L, MT_NAME);
  lua_setmetatable(L, 1);
  ud->ctx = ctx;
  ud->dev = dev;
  ud->devh = devh;
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
    return 0;
  }
  
  return 1;
}

static const struct luaL_Reg uvc_methods [] = {
  {"stream_on", lua_uvc2_stream_on},
  {"stream_off", lua_uvc2_stream_off},
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
