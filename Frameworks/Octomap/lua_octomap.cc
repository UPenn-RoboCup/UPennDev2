#include "lua_octomap.hh"

using namespace std;
using namespace octomap;

// Initialize the tree
OcTree tree (0.05);  
// Set the origin
//point3d origin (0.01f, 0.01f, 0.02f);
point3d origin (0.0f, 0.0f, 0.0f);

static int lua_add_scan( lua_State *L ) {

  /* Grab the points from the last laser scan*/
  const THFloatTensor * points_t = (THFloatTensor *) luaT_checkudata(L, 1, "torch.FloatTensor");
  const long nps = points_t->size[1]; // The number of laser points to match
  //fprintf(stdout,"Number of laser points: %ld\n", nps );
  //fflush(stdout);

  Pointcloud cloud;
  float x,y,z;
  /*
  x = THTensor_fastGet2d( points_t, 0, 500);
  y = THTensor_fastGet2d( points_t, 1, 500);
  z = THTensor_fastGet2d( points_t, 2, 500);
  fprintf(stdout,"Inserting %f %f %f\n",x,y,z);
  */
  for (long p=0; p<nps; p++) {
    x = THTensor_fastGet2d( points_t, 0, p);
    y = THTensor_fastGet2d( points_t, 1, p);
    z = THTensor_fastGet2d( points_t, 2, p);
    point3d observed_point ( x, y, z );
    // Ray cast to make free space
    if (!tree.insertRay(origin, origin+observed_point)) {
      // Lua_error?
      cout << "ERROR while inserting ray from " << origin << " to " << observed_point << endl;
    }
    // Add point to the cloud which is inserted.
    // Is this the most efficient way?
    cloud.push_back(observed_point);
  }  
  // insert in global coordinates:
  tree.insertPointCloud(cloud, origin);
  return 0;
}

// Save to a file
static int lua_save_tree( lua_State *L ) {
  cout << "writing to spherical_scan.bt..." << endl;
  tree.writeBinary("spherical_scan.bt");
  printf("\n");
  return 0;
}

static const struct luaL_reg octomap_lib [] = {
  {"add_scan", lua_add_scan},
  {"save_tree", lua_save_tree},
  {NULL, NULL}
};

int luaopen_Octomap(lua_State *L) {
  luaL_register(L, "Octomap", octomap_lib);
  return 1;
}
