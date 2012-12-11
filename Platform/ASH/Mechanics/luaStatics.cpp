#include "luaKDL_utils.h"
#include "luaStatics.h"
#include "Mechanics.h"

using namespace KDL;

extern _Mechanics Mechanics;

static int lua_get_cog(lua_State *L)
{
  // TODO
  return 0;
}

static const struct luaL_reg Statics_lib [] = {
  {"get_cog", lua_get_cog},
};

extern "C"
int luaopen_Statics(lua_State *L)
{
  luaL_register(L, "Statics", Statics_lib);
  return 1;
}
