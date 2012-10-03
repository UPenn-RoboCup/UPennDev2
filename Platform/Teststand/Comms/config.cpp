#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/allocators/allocator.hpp>
#include "lua.h"
#include "lualib.h"
#include "lauxlib.h"
#include "config.h"

// config : global interface for config data
// author : Mike Hopkins
///////////////////////////////////////////////////////////////////////////

// create global Config instance
Config config;

static std::vector<double> lua_getvector(lua_State *L, int index);

Config::Config()
{
  // load config values from Config.lua module

  lua_State *L = luaL_newstate();
  luaL_openlibs(L);
  if (luaL_loadfile(L, "include.lua") || lua_pcall(L, 0, 0, 0))
  {
    fprintf(stderr, "%s", lua_tostring(L, -1));
    exit(EXIT_FAILURE);
  }
  if (luaL_loadstring(L, "require('Config')") || lua_pcall(L, 0, 0, 0))
  {
    fprintf(stderr, "%s", lua_tostring(L, -1));
    exit(EXIT_FAILURE);
  }
  lua_getglobal(L, "Config");
  if (!lua_istable(L, -1))
  {
    fprintf(stderr, "unable to load Config table from Config.lua");
    exit(EXIT_FAILURE);
  }
  lua_getfield(L, -1, "bias");
  if (!lua_istable(L, -1))
  {
    fprintf(stderr, "unable to load Config.bias table from Config.lua");
    exit(EXIT_FAILURE);
  }

  lua_getfield(L, -1, "joint_position_bias");
  joint_position_bias = lua_getvector(L, -1);
  joint_position_bias.resize(N_JOINT, 0);
  lua_pop(L, 1);

  lua_getfield(L, -1, "joint_force_bias");
  joint_force_bias = lua_getvector(L, -1);
  joint_force_bias.resize(N_JOINT, 0);
  lua_pop(L, 1);

  lua_getfield(L, -1, "motor_position_bias");
  motor_position_bias = lua_getvector(L, -1);
  motor_position_bias.resize(N_MOTOR, 0);
  lua_pop(L, 1);

  lua_getfield(L, -1, "motor_force_bias");
  motor_force_bias = lua_getvector(L, -1);
  motor_force_bias.resize(N_MOTOR, 0);
  lua_pop(L, 1);

  lua_getfield(L, -1, "force_torque_bias");
  force_torque_bias = lua_getvector(L, -1);
  force_torque_bias.resize(N_FORCE_TORQUE, 0);
  lua_pop(L, 1);

  lua_getfield(L, -1, "tactile_array_bias");
  tactile_array_bias = lua_getvector(L, -1);
  tactile_array_bias.resize(N_TACTILE_ARRAY, 0);
  lua_pop(L, 1);

  lua_close(L);
}

Config::~Config()
{
}

std::vector<double> lua_getvector(lua_State *L, int index)
{
  // get double vector from lua stack
  std::vector<double> vec(0);
  if (!lua_istable(L, index))
    return vec;
  size_t len = lua_objlen(L, index);
  vec.resize(len);
  if (index < 0)
    --index;

  for (int i = 0; i < len; i++)
  {
    lua_pushinteger(L, i + 1);
    lua_gettable(L, index);
    vec[i] = luaL_checknumber(L, -1);
    lua_pop(L, 1);
  }
  return vec;
}
