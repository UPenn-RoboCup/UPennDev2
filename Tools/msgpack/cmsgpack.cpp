/*
 * Copyright 2002-2010 Guillaume Cottenceau.
 *
 * This software may be freely redistributed under the terms
 * of the X11 license.
 * 
 * lua wrapper by Yida Zhang <yida@seas.upenn.edu>
 * University of Pennsylvania
 */

#ifdef __cplusplus
extern "C"
{
#endif
#include "lua.h"
#include "lualib.h"
#include "lauxlib.h"
#ifdef __cplusplus
}
#endif

#include <msgpack.hpp>
#include <iostream>
#include <string>

static int lua_cmsgpack_new(lua_State *L) {
  const char* str = luaL_checkstring(L, 1);
  int size = lua_tointeger(L, 2);
  msgpack::unpacked msg;
  msgpack::unpack(&msg, str, size);

  msgpack::object obj = msg.get();
  std::cout << obj << std::endl;

  return 1;
}

static const struct luaL_reg cmsgpack_Functions [] = {
  {"new", lua_cmsgpack_new},
  {NULL, NULL}
};

#ifdef __cplusplus
extern "C"
#endif
int luaopen_cmsgpack(lua_State *L) {

  luaL_register(L, "cmsgpack", cmsgpack_Functions);
  return 1;
}
