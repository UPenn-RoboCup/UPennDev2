#ifdef __cplusplus
extern "C" {
#endif

#include "lua.h"
#include "lualib.h"
#include "lauxlib.h"

#ifdef __cplusplus
}
#endif

#include "OccMap.h"
#include <iostream>
#include <iomanip>
#include <vector>
#include <cassert>

OccMap map;


static int lua_occmap_reset(lua_State *L) {
  map.randomize_map();
  return 1;
}

static int lua_occmap_init_map(lua_State *L) {
  int map_size = luaL_checkint(L, 1);
  int robot_x = luaL_checkint(L, 2);
  int robot_y = luaL_checkint(L, 3);
  double time = luaL_checknumber(L, 4);
  map.reset_size(map_size, robot_x, robot_y, time);
  // Init Odometry 
  map.odometry_init();
  return 1;
}

static int lua_occmap_init_vision_proc(lua_State *L) {
  int observation_width = luaL_checkint(L, 1);
  map.vision_proc_init(observation_width);
  return 1;
}

static int lua_occmap_odometry_update(lua_State *L) {
  double odomX = luaL_checknumber(L, 1);
  double odomY = luaL_checknumber(L, 2);
  double odomA = luaL_checknumber(L, 3);
  map.odometry_update(odomX, odomY, odomA);
  return 1;
}

static int lua_occmap_vision_update(lua_State *L) {
  if (!lua_istable(L, 1))
    return luaL_error(L, "Input freespace bound not table data");
  vector<double> free_bound;
  lua_pushnil(L);
  while (lua_next(L, 1) != 0) {
    free_bound.push_back(lua_tonumber(L, -1));
    lua_pop(L, 1);
  }
  if (!lua_istable(L, 2))
    return luaL_error(L, "Input freespace bound type not table data");
  vector<int> free_bound_type;
  lua_pushnil(L);
  while (lua_next(L, 2) !=0) {
    free_bound_type.push_back(lua_tointeger(L, -1));
    lua_pop(L, 1);
  }
  int width = luaL_checkint(L, 3);
  if (free_bound.size() != 2 * width) return 0;
  if (free_bound_type.size() != 2 * width) return 0;
  double time = luaL_checknumber(L, 4);
  map.vision_update(free_bound, free_bound_type, width, time);
  return 1;
}

static int lua_occmap_retrieve_map(lua_State *L) {
	lua_pushlightuserdata(L, &map.get_map()[0]);
  return 1;
}

static int lua_occmap_retrieve_map_updated_time(lua_State *L) {
	lua_pushlightuserdata(L, &map.get_map_updated_time()[0]);
  return 1;
}

static int lua_occmap_retrieve_data(lua_State *L) {
  lua_createtable(L, 0, 1);

  lua_pushstring(L, "robot_pos");
  lua_createtable(L, 2, 0);
  lua_pushinteger(L, map.get_robot_pos_x());
  lua_rawseti(L, -2, 1);
  lua_pushinteger(L, map.get_robot_pos_y());
  lua_rawseti(L, -2, 2);
  lua_settable(L, -3);

  return 1;
}

static int lua_occmap_empty_userdata(lua_State *L) {
  int size = luaL_checkint(L, 1);
  vector<double> empty;
  empty.resize(size);
  lua_pushlightuserdata(L, &empty[0]);
  return 1;
}

static int lua_occmap_time_decay(lua_State *L) {
  double time = luaL_checknumber(L, 1);
  map.time_decay(time);
  return 1;
}

static const struct luaL_reg OccMap_lib [] = {
  {"init", lua_occmap_init_map},
  {"vision_init", lua_occmap_init_vision_proc},
  {"reset", lua_occmap_reset},
  {"time_decay", lua_occmap_time_decay},
  {"retrieve_map", lua_occmap_retrieve_map},
  {"retrieve_map_updated_time", lua_occmap_retrieve_map_updated_time},
  {"retrieve_data", lua_occmap_retrieve_data},
  {"vision_update", lua_occmap_vision_update},
  {"odometry_update", lua_occmap_odometry_update},
  {"empty_userdata", lua_occmap_empty_userdata},
  {NULL, NULL}
};

extern "C"
int luaopen_OccMap (lua_State *L) {
  luaL_register(L, "OccMap", OccMap_lib);
  return 1;
}
