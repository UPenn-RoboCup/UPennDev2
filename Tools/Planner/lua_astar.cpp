/*
  astar planning library for lua
  
  Daniel D. Lee (ddlee@seas.upenn.edu), 4/2007
  Lua wrapper by Yida Zhang (yida@seas.upenn.edu), 4/2013
*/

#include <lua.hpp>

static const luaL_Reg astar_functions [] = {
  {NULL, NULL}
};

static const luaL_Reg astar_methods [] = {
  {NULL, NULL}
};

#ifdef __cplusplus
extern "C"
#endif
int luaopen_astar(lua_State *L) {
#if LUA_VERSION_NUM == 502
  luaL_newlib(L, astar_functions);
#else
  luaL_register(L, "astar", astar_functions);
#endif

  return 1;
}
