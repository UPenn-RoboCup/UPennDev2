#include <vector>
#include <stdint.h>
#include <octomap/octomap.h>
// #include <octomap/octomap_types.h>
#include <octomap/math/Utils.h>

struct group {
  int count = 0;  
  octomap::point3d mean_normal = octomap::point3d(0.0f, 0.0f, 0.0f);
  octomap::point3d sum_normal = octomap::point3d(0.0f, 0.0f, 0.0f);
  std::vector<octomap::OcTreeKey> members;
};

group *normal_clustering(octomap::OcTree tree, int n);

