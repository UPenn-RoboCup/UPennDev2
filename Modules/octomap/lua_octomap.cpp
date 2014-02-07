/*
 Lua Octomap Wrapper
(c) 2013 Stephen McGill
*/
#include <lua.hpp>
#include "torch/luaT.h"
#include "torch/TH/TH.h"

#include <stdio.h>
#include <vector>
#include <string>

#include <octomap/octomap.h>
#include <octomap/math/Utils.h>

#include <sstream>

using namespace std;
using namespace octomap;

// TODO: Make a metatable with a pointer to each tree

/* Initialize the tree (5mm) */
static OcTree tree (0.005);
/* Gloabl Origin */
static point3d origin (0.0f,0.0f,0.3f);

static int lua_set_origin( lua_State *L ) {
	static float ox,oy,oz;
	const THDoubleTensor * origin_t =
		(THDoubleTensor *) luaT_checkudata(L, 1, "torch.DoubleTensor");
	ox = (float)THTensor_fastGet1d( origin_t, 0);
	oy = (float)THTensor_fastGet1d( origin_t, 1);
	oz = (float)THTensor_fastGet1d( origin_t, 2);
	origin = point3d(ox,oy,oz);
	return 0;
}

static int lua_add_scan( lua_State *L ) {

	/* Grab the points from the last laser scan*/
	const THDoubleTensor * points_t =
		(THDoubleTensor *) luaT_checkudata(L, 1, "torch.DoubleTensor");
	const long nps = points_t->size[0]; // The number of laser points to match
	/* Check to optionally use raycasting from the origin */
	int use_raycast = luaL_optint(L, 2, 0);
	
	float x,y,z;
	Pointcloud cloud;
	for (long p=0; p<nps; p++) {
		x = THTensor_fastGet2d( points_t, p, 0 );
		y = THTensor_fastGet2d( points_t, p, 1 );
		z = THTensor_fastGet2d( points_t, p, 2 );
		/* Add point to the cloud which is inserted */
		// TODO: Is this the most efficient way?
		cloud.push_back(x,y,z);
	}  
	// insert in global coordinates:
	tree.insertPointCloud(cloud, origin);
	
	/*
	lua_pushnumber(L, tree.memoryUsage());
	return 1;
	*/
	return 0;
}

static int lua_get_pruned_data( lua_State *L ) {
	stringstream ss;
	if ( !tree.writeBinary( ss ) )
		return luaL_error(L,"Bad writing of the octomap!");
	string tree_str = ss.str();
	lua_pushlstring(L, tree_str.c_str(), tree_str.length() );
	return 1;
}

static int lua_get_data( lua_State *L ) {
	stringstream ss;
	if ( !tree.writeBinaryConst( ss ) )
		return luaL_error(L,"Bad writing of the octomap!");
	string tree_str = ss.str();
	lua_pushlstring(L, tree_str.c_str(), tree_str.length() );
	return 1;
}

static int lua_save_tree( lua_State *L ) {
	const char* filename = luaL_checkstring(L,1);
	tree.writeBinary(filename);
	return 0;
}

static const struct luaL_Reg octomap_lib [] = {
	{"add_scan", lua_add_scan},
	{"set_origin", lua_set_origin},
	{"get_data", lua_get_data},
	{"get_pruned_data", lua_get_pruned_data},
	{"save_tree", lua_save_tree},
	{NULL, NULL}
};

extern "C" int luaopen_octomap(lua_State *L) {
#if LUA_VERSION_NUM == 502
	luaL_newlib(L, octomap_lib);
#else
	luaL_register(L, "octomap", octomap_lib);
#endif
	return 1;
}
