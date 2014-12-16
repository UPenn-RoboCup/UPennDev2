#include <vector>
#include <stdint.h>
#include <octomap/octomap.h>
#include <octomap/math/Utils.h>

#include "normal_clustering.h"

//TODO: a class for plane??

int ransac(std::vector<octomap::OcTreeKey> &bestInlierKey, std::vector<group> &groups, 
  octomap::OcTree tree, int n, float eps);

