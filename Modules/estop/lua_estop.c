#include "lua.h"
#include "lualib.h"
#include "lauxlib.h"

#include <wchar.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#include "SerialInterface.h"
#include "VehicleInterface.h"
#include "estop.h"

#define MT_NAME "estop_mt"



static int lua_estop_index(lua_State *L) {
  /* Get index through metatable: */
  if (!lua_getmetatable(L, 1)) {lua_pop(L, 1); return 0;} /* push metatable */
  lua_pushvalue(L, 2); /* copy key */
  lua_rawget(L, -2); /* get metatable function */
  lua_remove(L, -2); /* delete metatable */
  return 1;
}

static int lua_estop_delete(lua_State *L) { 
  estop_shutdown();
  return 1;
}

static int lua_estop_init(lua_State *L) {
  estop_init();
  return 1;
}


static int lua_estop_update(lua_State *L) {
  estop_update();
  return 1;
}


static const struct luaL_reg estop_functions [] = {
  {"init", lua_estop_init},
  {"update", lua_estop_update},
  {NULL, NULL}
};

static const struct luaL_reg estop_methods [] = {
  //{"get_raw", lua_spnav_get_raw},
  //{"get", lua_spnav_get},
  {"__gc", lua_estop_delete},  
  {"__index", lua_estop_index},
  {NULL, NULL}
};

int luaopen_estop(lua_State *L) {
  luaL_newmetatable(L, MT_NAME);

#if LUA_VERSION_NUM == 502
  luaL_setfuncs(L, estop_methods, 0);
  luaL_newlib(L, estop_functions);
#else
  luaL_register(L, NULL, estop_methods);
  luaL_register(L, "estop", estop_functions);
#endif

  return 1;
}
