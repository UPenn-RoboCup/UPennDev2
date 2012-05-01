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
static std::vector<float> map;

static int lua_occmap_init(lua_State *L) {
	n = luaL_checkint(L, 1);
	float ran_num = 0;
	srand(time(0));
	for (int i = 0; i < n*n; i++) {
		ran_num = rand() / 1.0 / RAND_MAX;
		ran_num = std::log(ran_num/(1-ran_num));
		map.push_back(ran_num);
	}
	return 1;
}

static int lua_occmap_retrieve(lua_State *L) {
	lua_pushstring(L, "map");
	lua_createtable(L, map.size(), 0);
	for (int i = 0; i < map.size(); i++) {
		lua_pushnumber(L, map[i]);
		lua_rawseti(L, -2, i+1);
	}
	return 1;
}

static const struct luaL_reg OccMap_lib [] = {
  {"init", lua_occmap_init},
	{"retrieve", lua_occmap_retrieve},
  {NULL, NULL}
};

#ifdef __cplusplus
extern "C"
#endif
int luaopen_OccMap (lua_State *L) {
  luaL_register(L, "OccMap", OccMap_lib);
  return 1;
}
