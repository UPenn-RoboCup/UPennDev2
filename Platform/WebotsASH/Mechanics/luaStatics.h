#ifndef _LUA_STATICS_H_
#define _LUA_STATICS_H_

extern "C"
{
#include <lua.h>
#include <lualib.h>
#include <lauxlib.h>
}

extern "C"
int luaopen_Statics(lua_State *L);

#endif
