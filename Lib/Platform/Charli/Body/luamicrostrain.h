#ifndef LUA_MICROSTRAIN_H_
#define LUA_MICROSTRAIN_H_

extern "C"
{
#include <lua.h>
#include <lualib.h>
#include <lauxlib.h>
}

extern "C"
int luaopen_microstrain(lua_State *L);

#endif
