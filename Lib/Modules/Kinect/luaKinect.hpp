#ifndef luaKinect_h_DEFINED
#define luaKinect_H_DEFINED

extern "C"
{
#include "lua.h"
#include "lualib.h"
#include "lauxlib.h"
}

#include <stdio.h>
#include <OpenNI.h>
#include <vector>
#include <string>

extern "C"
int luaopen_Kinect(lua_State *L);

#endif
