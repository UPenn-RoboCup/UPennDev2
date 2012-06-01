#include "SoundComm.h"
#include "sound_comm_thread.h"

#include <string>

static int lua_test(lua_State *L) {
  lua_pushinteger(L, 4);
  return 1;
}

static const struct luaL_reg sound_comm_lib [] = {
  {"test", lua_test},


  {NULL, NULL}
};

extern "C"
int luaopen_SoundComm (lua_State *L) {
  luaL_register(L, "SoundComm", sound_comm_lib);
  
  return 1;
}
