#include "SoundComm.h"
#include "sound_comm_thread.h"

// defined in nao_comm_thread.cc
// TODO: detection list

int init = 0;

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
  if (!init) {
    if (sound_comm_thread_init() != 0) {
      printf("SoundComm: error initializing.\n");
      exit(1);
    } 
    init = 1;
  }
  
  return 1;
}

