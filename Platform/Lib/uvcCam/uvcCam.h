/*
  Author: Daniel D. Lee <ddlee@seas.upenn.edu>, 05/10
  	: Stephen McGill 10/10
*/
			
#ifndef luauvcCam_h_DEFINED
#define luauvcCam_H_DEFINED

#include "timeScalar.h"
#include "v4l2.h"
#include <string.h>

#define VIDEO_DEVICE "/dev/video0"

extern "C"
{
#include "lua.h"
#include "lualib.h"
#include "lauxlib.h"
}

extern "C"
int luaopen_uvcCam(lua_State *L);

typedef struct {
  int count;
  int select;
  double time;
  double joint[20];
} CAMERA_STATUS;

/* Exposed C functions to Lua */
typedef unsigned char uint8;
typedef unsigned int uint32;

#endif
