#ifndef _LUA_KINEMATICS_H_
#define _LUA_KINEMATICS_H_

extern "C"
{
#include <lua.h>
#include <lualib.h>
#include <lauxlib.h>
}

extern "C"
int luaopen_Kinematics(lua_State *L);

#endif
