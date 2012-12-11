#ifndef _LUA_DYNAMICS_H_
#define _LUA_DYNAMICS_H_

extern "C"
{
#include <lua.h>
#include <lualib.h>
#include <lauxlib.h>
}

extern "C"
int luaopen_Dynamics(lua_State *L);

#endif
