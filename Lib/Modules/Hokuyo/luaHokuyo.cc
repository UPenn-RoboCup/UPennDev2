#include "luaHokuyo.h"

static const struct luaL_reg hokuyo_lib [] = {
  {NULL, NULL}
};

int luaopen_Hokuyo(lua_State *L) {
  luaL_register(L, "Hokuyo", hokuyo_lib);
  return 1;
}

