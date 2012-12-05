#ifdef __cplusplus
extern "C" {
#endif

#include "lua.h"
#include "lualib.h"
#include "lauxlib.h"

#ifdef __cplusplus
}
#endif

static const struct luaL_reg Slam_lib [] = {
  {NULL, NULL}
};



extern "C"
int luaopen_Slam (lua_State *L) {
  luaL_register(L, "Slam", Slam_lib);
  return 1;
}
