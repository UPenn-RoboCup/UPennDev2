/*
 Lua Octomap Wrapper
(c) 2014 Stephen McGill, Qin He
*/

#include <lua.hpp>
#include <torch/luaT.h>
#include <torch/TH/TH.h>

#include <stdio.h>
#include <vector>
#include <string>
#include <time.h>

#include <octomap/octomap.h>
#include <octomap/math/Utils.h>

#include <sstream>

using namespace std;
using namespace octomap;

// TODO: Make a metatable with a pointer to each tree\

#define DEFAULT_RESOLUTION 0.01
#define DEFAULT_MIN_RANGE 0.05
#define DEFAULT_MAX_RANGE 10

// For timing
clock_t t0, t1;

double min_range = DEFAULT_MIN_RANGE; //TODO: do we use it?
double max_range = DEFAULT_MAX_RANGE;

/* Initialize the tree (1cm) */
static OcTree tree (DEFAULT_RESOLUTION);
/* Gloabl Origin */
static point3d origin (0.0f,0.0f,0.3f);


static int lua_set_resolution(lua_State *L) {
  double res = luaL_checknumber(L, 1);
  tree.setResolution(res);
  return 0;
}

static int lua_set_range(lua_State *L) {
  min_range = luaL_checknumber(L, 1);
  max_range = luaL_checknumber(L, 2);
  return 0;
}

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

//TODO: reset map
static int lua_reset_octomap() {
  return 0;
}


static int lua_add_depth(lua_State *L) {
	/* Grab the depth image*/
  const THFloatTensor * depths_t =
    (THFloatTensor *) luaT_checkudata(L, 1, "torch.FloatTensor");
  float focal_len = (float) luaL_checknumber(L, 2);
  const long nr = depths_t->size[0];
  const long nc = depths_t->size[1];
  // Check contiguous
  THArgCheck(depths_t->stride[1] == 1, 1, "Depth not contiguous (j)");
  THArgCheck(depths_t->stride[0] == nc, 1, "Improper depth layout (i)");
  // Get the pose
  float px = (float) luaL_checknumber(L, 3);
  float py = (float) luaL_checknumber(L, 4);
  float pz = (float) luaL_checknumber(L, 5);
  double roll = (double) luaL_checknumber(L, 6);
  double pitch = (double) luaL_checknumber(L, 7);
  double yaw = (double) luaL_checknumber(L, 8);
  pose6d kinect_pose = pose6d(px,py,pz,roll,pitch,yaw);
  
  /*
  X = d
  Y = -(x-cx)*d/f
  Z = -(y-cy)*d/f
  */
  //TODO: cx, cy need to be calibrated
  float cx = (float) nc/2, cy = (float) nr/2;
  float inv_f = 1 / focal_len;
  float x, y, z;
  float *depths_ptr = (float *)(depths_t->storage->data + depths_t->storageOffset);
  Pointcloud cloud;

  // t0 = clock();
  for (int j=0; j<nr; j++) {
    for (int i=0; i<nc; i++) {
      x = *(depths_ptr);
      y = -(i-cx) * x * inv_f;
      z = -(j-cy) * x * inv_f;
      depths_ptr++;
      // Add to cloud
      cloud.push_back(x,y,z);
    }
  }
  // t1 = clock();
  // printf("(%f seconds) creating cloud\n", (float)(t1-t0)/CLOCKS_PER_SEC);
  
  t0 = clock();
	// Update tree chunk by chunk
  tree.insertPointCloud(cloud, point3d(0.0f,0.0f,0.0f), kinect_pose, max_range, true);
  tree.updateInnerOccupancy();
  tree.prune();
  t1 = clock();
  printf("(%f seconds) inserting cloud\n", (float)(t1-t0)/CLOCKS_PER_SEC);	
  printf("Mem for tree: %f MB\n\n",(double)tree.memoryUsage()/1E6);
  printf("tree depth: %u, \t # of leaves:%u \n\n", tree.getTreeDepth(), tree.getNumLeafNodes());
    
  return 0;
}

static int lua_add_scan( lua_State *L ) {
	/* Grab the points from the last laser scan*/
	const THFloatTensor * points_t =
		(THFloatTensor *) luaT_checkudata(L, 1, "torch.FloatTensor");
	// Check contiguous
	THArgCheck(points_t->stride[1] == 1, 1, "Point cloud not contiguous (j)");  
	const long nps = points_t->size[0];
	const long points_istride = points_t->stride[0];
	THArgCheck(points_istride == points_t->size[1], 1, "Improper point cloud layout (i)"); 
  
	/* Check to optionally use raycasting from the origin */
  // int use_raycast = luaL_optint(L, 2, 0);
	
  float *points_ptr = (float *)(points_t->storage->data + points_t->storageOffset);  
	float x,y,z;
	Pointcloud cloud;
  
  t0 = clock();  
	for (long p=0; p<nps; p++) {
    x = *(points_ptr);
    y = *(points_ptr+1);
    z = *(points_ptr+2);
    points_ptr += points_istride;
    
    // update tree ray by ray
    // tree.insertRay(origin, point3d(x,y,z), max_range, true);
    
		/* Add point to the cloud */
    cloud.push_back(x,y,z);
	}
  t1 = clock();
  printf("(%f seconds) creating cloud\n", (float)(t1-t0)/CLOCKS_PER_SEC);
  
  
  t0 = clock();
	// Update tree chunk by chunk
  tree.insertPointCloud(cloud, origin, max_range, true);
  tree.updateInnerOccupancy();
  tree.prune();
  t1 = clock();
  printf("(%f seconds) inserting cloud\n", (float)(t1-t0)/CLOCKS_PER_SEC);	
  printf("Mem for tree: %f MB\n\n",(double)tree.memoryUsage()/1E6);
  // printf("tree depth: %u, \t # of leaves:%u \n\n", tree.getTreeDepth(), tree.getNumLeafNodes());
  
	/*
	lua_pushnumber(L, tree.memoryUsage());
	return 1;
	*/
	return 0;
}



// Our primer normals
const point3d ground_normal = point3d(0.0f, 0.0f, 1.0f);

// Get the horizontal nodes in givein bbox
// static vector<OcTreeKey> get_horizontal(lua_State *L) {
static int lua_get_horizontal(lua_State *L) {
  float min_x, min_y, min_z;
  float max_x, max_y, max_z;
  min_x = luaL_checknumber(L, 1);  //TODO
  min_y = luaL_checknumber(L, 2);
  min_z = luaL_checknumber(L, 3);
  max_x = luaL_checknumber(L, 4);
  max_y = luaL_checknumber(L, 5);
  max_z = luaL_checknumber(L, 6);
  

  vector<OcTreeKey> candidates;
  // Bbox for searching the candidates
  point3d min = point3d(min_x, min_y, min_z);
  point3d max = point3d(max_x, max_y, max_z);
  vector<point3d> normals;
  for(OcTree::leaf_bbx_iterator it = tree.begin_leafs_bbx(min,max),
         end=tree.end_leafs_bbx(); it!= end; ++it) {
      
      if (fabs(it.getZ()-1)<0.05) {
        candidates.push_back(it.getKey());
      }

      // tree.getNormals(it.getCoordinate(), normals);
      // printf("# of Normals: %d\n", normals.size());
      // printf("point: %.2f %.2f %.2f\n", it.getX(), it.getY(), it.getZ());
      // printf("1st normal: %.2f %.2f %.2f\n", normals[1].x(), normals[1].y(),normals[1].z());
      
      // if (normals[1].angleTo(ground_normal) < DEG2RAD(2)) { //TODO:something wrong
      //   printf("HEIGHT IS: %d", it.getZ());
      //   candidates.push_back(it.getKey());
      // }

  }
  printf("# of cands %d\n", candidates.size());
  // return candidates;
  return 0;
}


//TODO: segmentaion thoughs: table, object, brackground


//TODO: Region growing
static void connected_region(vector<OcTreeKey> &keys, bool hor) {
  if (hor) {
    
  } else {
    
  }
  // Propagate
  // if it's a sinlge layer
  // if it's surrounded by 4 cells
  
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

// TODO: query nodes, connected components for plane fitting

static const struct luaL_Reg octomap_lib [] = {
  {"set_resolution", lua_set_resolution},
	{"set_origin", lua_set_origin},
  {"set_range", lua_set_range},
  //
	{"add_scan", lua_add_scan},
  {"add_depth", lua_add_depth},
  //
  {"get_horizontal", lua_get_horizontal},
  //
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
