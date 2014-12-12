#include <vector>
#include <stdint.h>
#include <octomap/octomap.h>
#include <octomap/math/Utils.h>

#include "normal_clustering.h"

//TODO: a class for plane??

int ransac(octomap::OcTree tree, std::vector<group> &groups, int n, float eps);

