#include <octomap/octomap.h>
#include <octomap/math/Utils.h>
#include "lua_octomap.hh"

using namespace std;
using namespace octomap;

// Initialize the tree
OcTree tree (0.05);  
// Set the origin
point3d origin (0.01f, 0.01f, 0.02f);
// Dummy point?
point3d point_on_surface (4.01f, 0.01f, 0.01f);

static int lua_add_scan( lua_State *L ) {
  Pointcloud cloud;

  cout << "generating Hokuyo scan at " << origin << " ..." << endl;
  // Hokuyo is -145 to 145
  for (int i=-145; i<145; i++) {

    // Make a temporary rotated point
    point3d rotated = point_on_surface;
    rotated.rotate_IP(0, DEG2RAD(i), 0 );

    // Ray cast to make free space
    if (!tree.insertRay(origin, origin+rotated)) {
      // Lua_error?
      cout << "ERROR while inserting ray from " << origin << " to " << point_on_surface << endl;
    }

    // Add point to the cloud which is inserted.
    // Is this the most efficient way?
    cloud.push_back(rotated);
  }  
  // insert in global coordinates:
  tree.insertPointCloud(cloud, origin);

  cout << "done." << endl;
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
