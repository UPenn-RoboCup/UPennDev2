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
  map.odometry_reset();
  return 1;
}

static int lua_occmap_init_map(lua_State *L) {
  int map_size = luaL_checkint(L, 1);
  int robot_x = luaL_checkint(L, 2);
  int robot_y = luaL_checkint(L, 3);
  double time = luaL_checknumber(L, 4);
  map.reset_size(map_size, robot_x, robot_y, time);
  return 1;
}

static int lua_occmap_odometry_update(lua_State *L) {
  double odomX = luaL_checknumber(L, 1);
  double odomY = luaL_checknumber(L, 2);
  double odomA = luaL_checknumber(L, 3);
  map.odometry_update(odomX, odomY, odomA);
  return 1;
}

//static int lua_occmap_odometry_reset(lua_State *L) {
//  map.odometry_reset();
//  return 1;
//}

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
  if (free_bound.size() != 2 * width) {
    cout << "WARN: freespace bound data size must be double width" 
          << free_bound.size() << endl;
    return 0;
  }
  if (free_bound_type.size() != width) {
    cout << "WARN: freespace type data size must be width" 
          << free_bound_type.size() << endl;
    return 0;
  }
  double time = luaL_checknumber(L, 4);
// cout << "Vision Udpate before" << endl;
  map.vision_update(free_bound, free_bound_type, width, time);
//  cout << "vision update" << endl;
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

static int lua_occmap_retrieve_odometry(lua_State *L) {
  double pose_x = 0.0, pose_y = 0.0, pose_a = 0.0;
  map.get_odometry(pose_x, pose_y, pose_a);
  
  lua_createtable(L, 0, 1);
  
  lua_pushstring(L, "x");
  lua_pushnumber(L, pose_x);
  lua_settable(L, -3);

  lua_pushstring(L, "y");
  lua_pushnumber(L, pose_y);
  lua_settable(L, -3);

  lua_pushstring(L, "a");
  lua_pushnumber(L, pose_a);
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

static int lua_occmap_obstacle(lua_State *L) {
  map.init_obstacle();
  map.kmean_clustering();
  int maxOb = 2;
  obstacle zeros_ob = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  obstacle ob = zeros_ob;
  int nobstacle = map.get_nobstacle();
//  cout << "obstacle number:" << nobstacle << endl; 
  lua_createtable(L, nobstacle + 1, 0);
  lua_pushnumber(L, nobstacle);
  lua_rawseti(L, -2, 1);
  for (int cnt = 0; cnt < maxOb; cnt++) {
//    if (cnt < nobstacle) 
//      ob = map.get_obstacle(cnt);
//    else 
      ob = zeros_ob;
    lua_createtable(L, 0, 3);
    // centroid field
    //    std::cout << "centroid:" << obstacle.centroid_x << ' ' << obstacle.centroid_y << endl; 
    lua_pushstring(L, "centroid");
    lua_createtable(L, 2, 0);
    lua_pushnumber(L, ob.centroid_x);
    lua_rawseti(L, -2, 1);
    lua_pushnumber(L, ob.centroid_y);
    lua_rawseti(L, -2, 2);
    lua_settable(L, -3);

    // angle range field
    //    std::cout << "range:" << obstacle.left_angle_range * 180 / M_PI 
    //              << ' ' << obstacle.right_angle_range * 180 / M_PI << endl;    lua_pushstring(L, "angle_range");
    lua_pushstring(L, "angle_range");
    lua_createtable(L, 2, 0);
    lua_pushnumber(L, ob.left_angle_range);
    lua_rawseti(L, -2, 1);
    lua_pushnumber(L, ob.right_angle_range);
    lua_rawseti(L, -2, 2);
    lua_settable(L, -3);

    // nearest point field
    //    std::cout << "nearest:" << obstacle.nearest_x << ' ' << obstacle.nearest_y
    //              << ' ' << obstacle.nearest_dist << endl; 
    lua_pushstring(L, "nearest");
    lua_createtable(L, 3, 0);
    lua_pushnumber(L, ob.nearest_x);
    lua_rawseti(L, -2, 1);
    lua_pushnumber(L, ob.nearest_y);
    lua_rawseti(L, -2, 2);
    lua_pushnumber(L, ob.nearest_dist);
    lua_rawseti(L, -2, 3);
    lua_settable(L, -3);

    lua_rawseti(L, -2, cnt+1+1);
  }
  return 1;
}

static const struct luaL_reg OccMap_lib [] = {
  {"init", lua_occmap_init_map},
  {"reset", lua_occmap_reset},
  {"time_decay", lua_occmap_time_decay},
  {"retrieve_map", lua_occmap_retrieve_map},
  {"retrieve_map_updated_time", lua_occmap_retrieve_map_updated_time},
  {"retrieve_data", lua_occmap_retrieve_data},
  {"vision_update", lua_occmap_vision_update},
  {"odometry_update", lua_occmap_odometry_update},
//  {"odometry_reset", lua_occmap_odometry_reset},
  {"retrieve_odometry", lua_occmap_retrieve_odometry},
  {"empty_userdata", lua_occmap_empty_userdata},
  {"get_obstacle", lua_occmap_obstacle},
  {NULL, NULL}
};

extern "C"
int luaopen_OccMap (lua_State *L) {
  luaL_register(L, "OccMap", OccMap_lib);
  return 1;
}
