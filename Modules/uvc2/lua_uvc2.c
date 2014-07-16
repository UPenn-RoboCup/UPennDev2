/*
  Author: Stephen McGill 10/10
*/

#include <lua.h>
#include <lualib.h>
#include <lauxlib.h>
#include <libuvc/libuvc.h>
#include "timeScalar.h"

#include <stdint.h>
#include <stdio.h>

/* metatable name for uvc */
#define MT_NAME "uvc_mt"

void cb(uvc_frame_t *frame, void *ptr) {

}

static int lua_uvc_init(lua_State *L) {
    
  uvc_context_t *ctx;
  uvc_device_t *dev;
  uvc_device_handle_t *devh;
  uvc_stream_ctrl_t ctrl;
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
  /* Try to negotiate a 640x480 30 fps YUYV stream profile */
  res = uvc_get_stream_ctrl_format_size(
      devh, &ctrl, /* result stored in ctrl */
      UVC_FRAME_FORMAT_YUYV, /* YUV 422, aka YUV 4:2:2. try _COMPRESSED */
      640, 480, 30 /* width, height, fps */
  );
  /* Print out the result */
  uvc_print_stream_ctrl(&ctrl, stderr);
  if (res < 0) {
    /* device doesn't provide a matching stream */
    uvc_perror(res, "get_mode");
    return 0;
  }
  /* Start the video stream. The library will call user function cb:
   *   cb(frame, (void*) 12345)
   */
  res = uvc_start_streaming(devh, &ctrl, cb, 12345, 0);
  if (res < 0) {
    /* unable to start stream */
    uvc_perror(res, "start_streaming");
    return 0;
  }
  uvc_set_ae_mode(devh, 1); /* e.g., turn on auto exposure */
  /* End the stream. Blocks until last callback is serviced */
  uvc_stop_streaming(devh);
  
  /* Release our handle on the device */
  uvc_close(devh);
  
  /* Release the device descriptor */
  uvc_unref_device(dev);
  
  /* Close the UVC context. This closes and cleans up any existing device handles,
   * and it closes the libusb context if one was not provided. */
  uvc_exit(ctx);

  return 0;
}


static const struct luaL_Reg uvc_functions [] = {
    {"init", lua_uvc_init},
    {NULL, NULL}
};

#ifdef __cplusplus
extern "C"
#endif
int luaopen_uvc2 (lua_State *L) {
    /* create metatable for uvc module */
//    luaL_newmetatable(L, MT_NAME);

#if LUA_VERSION_NUM == 502
//    luaL_setfuncs(L, uvc_methods, 0);
	  luaL_newlib(L, uvc_functions);
#else 
//    luaL_register(L, NULL, uvc_methods);
    luaL_register(L, "uvc2", uvc_functions);
#endif
    return 1;
}
