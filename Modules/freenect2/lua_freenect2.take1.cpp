/*
 * This file is part of the OpenKinect Project. http://www.openkinect.org
 *
 * Copyright (c) 2011 individual OpenKinect contributors. See the CONTRIB file
 * for details.
 *
 * This code is licensed to you under the terms of the Apache License, version
 * 2.0, or, at your option, the terms of the GNU General Public License,
 * version 2.0. See the APACHE20 and GPL2 files for the text of the licenses,
 * or the following URLs:
 * http://www.apache.org/licenses/LICENSE-2.0
 * http://www.gnu.org/licenses/gpl-2.0.txt
 *
 * If you redistribute this file in source form, modified or unmodified, you
 * may:
 *   1) Leave this header intact and distribute it under the same terms,
 *      accompanying it with the APACHE20 and GPL20 files, or
 *   2) Delete the Apache 2.0 clause and accompany it with the GPL2 file, or
 *   3) Delete the GPL v2 clause and accompany it with the APACHE20 file
 * In all cases you must keep the copyright notice intact and include a copy
 * of the CONTRIB file.
 *
 * Binary distributions must follow the binary distribution requirements of
 * either License.
 */
/*
Lua Wrapper for some libfreenect2 functionality
(c) Stephen McGill 2013
*/
#include <lua.hpp>
#include <string>
#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/threading.h>

using namespace libfreenect2;

Freenect2 *freenect2 = NULL;
Freenect2Device *dev = NULL;
SyncMultiFrameListener *listener = NULL;
FrameMap frames;

/* Device state variables */
char init = 0;

static int lua_freenect_init(lua_State *L) {

#ifdef DEBUG	
  fprintf(stdout,"Starting up!\n");
  fflush(stdout);
#endif
  
  if(init==1)
    return luaL_error(L, "Two libfreenect2 devices not supported yet.\n" );

#ifdef DEBUG
  fprintf(stdout,"Initializing Freenect2...\n");
  fflush(stdout);
#endif
  
  freenect2 = new Freenect2();

#ifdef DEBUG
  fprintf(stdout,"Open Device...\n");
  fflush(stdout);
#endif
  
  dev = freenect2->openDefaultDevice();

#ifdef DEBUG
  fprintf(stdout,"Checking Device status...\n");
  fflush(stdout);
#endif
  
  if(dev == 0) {
    return luaL_error(L, "No device!");
  }

#ifdef DEBUG
  fprintf(stdout,"Starting listener...\n");
  fflush(stdout);
#endif
  
  listener = new SyncMultiFrameListener(
    Frame::Color | Frame::Ir | Frame::Depth
  );
  
#ifdef DEBUG
//  fprintf(stdout,"Instantiate frames...\n");
//  fflush(stdout);
#endif
  //frames = new FrameMap();
  
#ifdef DEBUG
  fprintf(stdout,"Color listener...\n");
  fflush(stdout);
#endif
  dev->setColorFrameListener(listener);
  
#ifdef DEBUG
  fprintf(stdout,"IR/Depth listener...\n");
  fflush(stdout);
#endif
  dev->setIrAndDepthFrameListener(listener);
  
#ifdef DEBUG
  fprintf(stdout, "Start device...\n");
  fflush(stdout);
#endif
  dev->start();

#ifdef DEBUG
  fprintf(stdout, "Get information...\n");
  fflush(stdout);
#endif  
  lua_pushstring(L, dev->getSerialNumber().c_str());
  lua_pushstring(L, dev->getFirmwareVersion().c_str());
  
  // Initialized!
  init = 1;
  
  return 2;
}

static int lua_freenect_update(lua_State *L) {

#ifdef DEBUG
  fprintf(stdout, "Wait for frame...\n");
  fflush(stdout);
#endif  
  listener->waitForNewFrame(frames);
  
#ifdef DEBUG
  fprintf(stdout, "Access frames...\n");
  fflush(stdout);
#endif  
  Frame *rgb = frames[Frame::Color];
  Frame *ir = frames[Frame::Ir];
  Frame *depth = frames[Frame::Depth];
  
#ifdef DEBUG
  fprintf(stdout, "Push frames...\n");
  fflush(stdout);
#endif  
  
  lua_newtable(L);
  lua_pushstring(L, "name");
  lua_pushstring(L, "color");
  lua_rawset(L, -3);
  lua_pushstring(L, "height");
  lua_pushnumber(L, rgb->height);
  lua_rawset(L, -3);
  lua_pushstring(L, "width");
  lua_pushnumber(L, rgb->width);
  lua_rawset(L, -3);
  lua_pushstring(L, "bpp");
  lua_pushnumber(L, rgb->bytes_per_pixel);
  lua_rawset(L, -3);
  // TODO: Push lstring instead?
  lua_pushstring(L, "data");
  lua_pushlightuserdata(L, rgb->data);
  lua_rawset(L, -3);
  
  lua_newtable(L);
  lua_pushstring(L, "name");
  lua_pushstring(L, "depth");
  lua_rawset(L, -3);
  lua_pushstring(L, "height");
  lua_pushnumber(L, depth->height);
  lua_rawset(L, -3);
  lua_pushstring(L, "width");
  lua_pushnumber(L, depth->width);
  lua_rawset(L, -3);
  lua_pushstring(L, "bpp");
  lua_pushnumber(L, depth->bytes_per_pixel);
  lua_rawset(L, -3);
  // TODO: Push lstring instead?
  lua_pushstring(L, "data");
  lua_pushlightuserdata(L, depth->data);
  lua_rawset(L, -3);
  
  lua_newtable(L);
  lua_pushstring(L, "name");
  lua_pushstring(L, "ir");
  lua_rawset(L, -3);
  lua_pushstring(L, "height");
  lua_pushnumber(L, ir->height);
  lua_rawset(L, -3);
  lua_pushstring(L, "width");
  lua_pushnumber(L, ir->width);
  lua_rawset(L, -3);
  lua_pushstring(L, "bpp");
  lua_pushnumber(L, ir->bytes_per_pixel);
  lua_rawset(L, -3);
  // TODO: Push lstring instead?
  lua_pushstring(L, "data");
  lua_pushlightuserdata(L, ir->data);
  lua_rawset(L, -3);
	
  return 3;
}

static int lua_freenect_shutdown(lua_State *L) {
	
  if(init==0)
    return luaL_error(L, "Cannot shutdown an uninitialized object!\n" );

#ifdef DEBUG
  printf("Stopping device\n");
  fflush(stdout);
#endif
  dev->stop();
  
#ifdef DEBUG
  printf("Closing device\n");
  fflush(stdout);
#endif
  dev->close();
  
  // Not initialized!
  init = 0;
  
  return 0;
}

static const struct luaL_Reg freenect2_lib [] = {
  {"init", lua_freenect_init},
  {"update", lua_freenect_update},
  {"shutdown", lua_freenect_shutdown},
  {NULL, NULL}
};

extern "C" int luaopen_freenect2(lua_State *L) {
#if LUA_VERSION_NUM == 502
  luaL_newlib(L, freenect2_lib);
#else
  luaL_register(L, "freenect2", freenect2_lib);
#endif
  return 1;
}
