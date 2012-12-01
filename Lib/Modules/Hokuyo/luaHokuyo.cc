#include "luaHokuyo.h"

#include <iostream>

static int lua_hokuyo_shutdown(lua_State *L) {
  std::cout << "Proper Shutdown Hokuyo" << std::endl;
  return 1;
}

static const struct luaL_reg hokuyo_lib [] = {
  {"shutdown", lua_hokuyo_shutdown}, 
  {NULL, NULL}
};

int luaopen_Hokuyo(lua_State *L) {
  luaL_register(L, "Hokuyo", hokuyo_lib);

  return 1;
}

