#ifndef LUA_STATICS_H 
#define LUA_STATICS_H 

extern "C"
{
#include <lua.h>
#include <lualib.h>
#include <lauxlib.h>
}

extern "C"
int luaopen_Statics(lua_State *L);

#endif
