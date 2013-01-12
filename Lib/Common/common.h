// This header is mainly for solving issues raised from lua 5.2 support
// This header must be placed after lua headers

#ifndef __LUA_COMMON_H__
#define __LUA_COMMON_H__

#if LUA_VERSION_NUM > 501
static void luaL_register(lua_State *L, const char *libname, const luaL_Reg *l) {
  luaL_newlib(L, l);
  lua_pushvalue(L, -1);
  lua_setglobal(L, libname);
}

#define luaL_reg luaL_Reg

#endif



#endif

