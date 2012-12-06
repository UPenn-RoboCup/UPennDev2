#ifdef __cplusplus
extern "C" {
#endif

#include "lua.h"
#include "lualib.h"
#include "lauxlib.h"

#ifdef __cplusplus
}
#endif

#include "lua_binStats.h"

static int lua_test(lua_State *L) {
  lua_createtable(L, 0, 1);
  lua_pushstring(L, "counter");
  lua_pushinteger(L, 5467);
  lua_settable(L, -3);

  lua_createtable(L, 0, 1);
  lua_pushstring(L, "ter");
  lua_pushinteger(L, 5467);
  lua_settable(L, -3);
 
  return 2;
}

static const struct luaL_reg Slam_lib [] = {
  {"test", lua_test},
  {"binStats", lua_binStats},
//  {"getMapCellsFromRay", lua_getMapCellsFromRay},
//  {"HoughTransform", lua_HoughTransform},
//  {"ScanMatch2D", lua_ScanMatch2D},
//  {"SubsambleDistance", lua_SubsambleDistance},
  {NULL, NULL}
};



extern "C"
int luaopen_Slam (lua_State *L) {
  luaL_register(L, "Slam", Slam_lib);
  return 1;
}


