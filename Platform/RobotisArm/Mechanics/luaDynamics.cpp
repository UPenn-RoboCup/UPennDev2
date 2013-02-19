#include "luaKDL_utils.h"
#include "luaDynamics.h"
#include "Mechanics.h"

using namespace KDL;

extern _Mechanics Mechanics;

static int lua_inverse_ds_legs(lua_State *L)
{
  // TODO
  return 0;
}

static int lua_inverse_ss_legs(lua_State *L)
{
  // TODO
  return 0;
}

static int lua_get_zmp(lua_State *L)
{
  // TODO
  return 0;
}

static const struct luaL_reg Dynamics_lib [] = {
  {"inverse_ds_legs", lua_inverse_ds_legs},
  {"inverse_ss_legs", lua_inverse_ss_legs},
  {"get_zmp", lua_get_zmp}
};

extern "C"
int luaopen_Dynamics(lua_State *L)
{
  luaL_register(L, "Dynamics", Dynamics_lib);
  return 1;
}
