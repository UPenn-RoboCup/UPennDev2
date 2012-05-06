#include <iostream>
#include <vector>
#include <ctime>
#include <cmath>
#include <cstdlib>

#ifdef __cplusplus
extern "C"
{
#endif
#include "lua.h"
#include "lualib.h"
#include "lauxlib.h"
#ifdef __cplusplus
}
#endif

// Default map size in terms of side blocks
static int n = 50;

// Map structure
static std::vector<double> map;

// Robot Centroid on Map
static int x_c = 25;
static int y_c = 25;

// Odometry init 
static double x = 0.0;
static double y = 0.0;
static double a = 0.0; 

double ran_num = 0;

static int lua_occmap_init(lua_State *L) {
	// Customerized map size
	n = luaL_checkint(L, 1);

	// Read robot centroid on map
	x_c = luaL_checkint(L, 2);
	y_c = luaL_checkint(L, 3);

	// init random seed based on time
	srand(time(NULL));
	for (int i = 0; i < n*n; i++) {
		ran_num = rand() / 1.0 / RAND_MAX;
		ran_num = std::log(ran_num/(1-ran_num));
		map.push_back(ran_num);
	}

	// Draw a square for debugging
	for (int i = n/3; i < n*2/3; i++) {
		int j = n / 3;
		map[j*n+i] = std::log(0.7/0.3);
		j = j * 2;
		map[j*n+i] = std::log(0.7/0.3);
	}
	for (int j = n/3; j < n*2/3; j++) {
		int i = n / 3;
		map[j*n+i] = std::log(0.7/0.3);
		i = i * 2;
		map[j*n+i] = std::log(0.7/0.3);
	}
	return 1;
}

static int lua_occmap_reset(lua_State *L) {
	// Reset Odometry 
	x = 0.0; y = 0.0; a = 0.0;
	// Reset Map
	map.clear();
	srand(time(NULL));
	for (int i = 0; i < n*n; i++) {
		ran_num = rand() / 1.0 / RAND_MAX;
		ran_num = std::log(ran_num/(1-ran_num));
		map.push_back(ran_num);
	}
	return 1;
}

static int lua_occmap_odometry_update(lua_State *L) {
	return 1;
}

static int lua_occmap_vision_update(lua_State *L) {
	return 1;
}

static int lua_occmap_retrieve(lua_State *L) {
	lua_createtable(L, 0, 2);

	lua_pushstring(L, "map");
	lua_createtable(L, map.size(), 0);
	for (int i = 0; i < map.size(); i++) {
		lua_pushnumber(L, map[i]);
		lua_rawseti(L, -2, i+1);
	}
	lua_settable(L, -3);

	lua_pushstring(L, "centroid");
	lua_createtable(L, 2, 0);
	lua_pushinteger(L, x_c);
	lua_rawseti(L, -2, 1);
	lua_pushinteger(L, y_c);
	lua_rawseti(L, -2, 2);
	lua_settable(L, -3);

	return 1;
}

static const struct luaL_reg OccMap_lib [] = {
  {"init", lua_occmap_init},
	{"retrieve", lua_occmap_retrieve},
	{"reset", lua_occmap_reset},
	{"vision_update", lua_occmap_vision_update},
	{"odometry_update", lua_occmap_odometry_update},
  {NULL, NULL}
};

#ifdef __cplusplus
extern "C"
#endif
int luaopen_OccMap (lua_State *L) {
  luaL_register(L, "OccMap", OccMap_lib);
  return 1;
}
