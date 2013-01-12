// This header is mainly for solving issues raised from lua 5.2 support
// This header must be placed after lua headers

#ifndef __LUA_COMMON_H__
#define __LUA_COMMON_H__

#if LUA_VERSION_NUM > 501
static void luaL_register(lua_State *L, const char *libname, const luaL_Reg *l) {
  luaL_newlib(L, l);
  lua_pushvalue(L, -1);
  if (libname != NULL)
    lua_setglobal(L, libname);
}

static int luaL_typerror(lua_State *L, int narg, const char *tname) {
  const char *msg = lua_pushfstring(L, "%s expected, got %s",
                                            tname, luaL_typename(L, narg));
  return luaL_argerror(L, narg, msg);
}

#define luaL_reg luaL_Reg
#define lua_objlen lua_rawlen 

#endif



#endif

