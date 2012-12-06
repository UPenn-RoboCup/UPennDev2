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
#include "lua_getMapCellsFromRay.h"
#include "lua_HoughTransform.h"
#include "lua_ScanMatch2D.h"
#include "lua_SubsambleDistance.h"


static const struct luaL_reg Slam_lib [] = {
  {"binStats", lua_binStats},
  {"getMapCellsFromRay", lua_getMapCellsFromRay},
  {"HoughTransform", lua_HoughTransform},
  {"ScanMatch2D", lua_ScanMatch2D},
  {"SubsambleDistance", lua_SubsambleDistance},
  {NULL, NULL}
};



extern "C"
int luaopen_Slam (lua_State *L) {
  luaL_register(L, "Slam", Slam_lib);
  return 1;
}


