#ifndef LUA_CBUFFER_H_
#define LUA_CBUFFER_H_

#include <lua.h>
#include <lualib.h>
#include <lauxlib.h>

#define lua_checkcbuffer(L, narg) \
  (cbuffer *)luaL_checkudata(L, narg, "cbuffer_mt")

int luaopen_cbuffer(lua_State *L);

#endif
