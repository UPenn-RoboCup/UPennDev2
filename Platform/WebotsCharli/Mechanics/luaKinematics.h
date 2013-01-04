#ifndef luaKinematics_H_DEFINED
#define luaKinematics_H_DEFINED

#include "CharliKinematics.h"

extern "C"
{
#include <lua.h>
#include <lualib.h>
#include <lauxlib.h>
}

extern "C"
int luaopen_Kinematics(lua_State *L);

#endif
