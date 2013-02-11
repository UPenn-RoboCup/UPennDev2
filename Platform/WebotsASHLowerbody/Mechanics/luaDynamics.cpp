#include "luaKDL_utils.h"
#include "luaDynamics.h"
#include "Mechanics.h"
#include <stdio.h>

using namespace KDL;

extern _Mechanics Mechanics;

static int lua_inverse(lua_State *L)
{
  // TODO
  return 0;
}

static int lua_cog(lua_State *L)
{
  // TODO
  return 0;
}

static const struct luaL_reg Dynamics_lib [] = {
  {"inverse", lua_inverse},
  {"cog", lua_cog}
};

extern "C"
int luaopen_Dynamics(lua_State *L)
{
  luaL_register(L, "Dynamics", Dynamics_lib);
  return 1;
}
