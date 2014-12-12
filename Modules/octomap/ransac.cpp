#include <algorithm>
#include "ransac.h"

using namespace std;
using namespace octomap;

int ransac(OcTree tree, vector<group> &groups, int maxIter, float eps) {
  // Params for RANSAC
  int n_inliers = 0, most_inliers = 0;
  point3d abc;
  float d = 0.0f;
  static vector<OcTreeKey> inliersKey, bestInliersKey;
  
  int n_points = groups[0].members.size();
  int i1=0, i2=0, i3=0;
  point3d p1, p2, p3;
  for (int i=0; i<maxIter; i++) {
    // Randomly pick three nodes
    while (i1==i2 || i2==i3 || i3==i1) {
      i1 = rand() % n_points;
      i2 = rand() % n_points;
      i3 = rand() % n_points; 
    }
    // Extract points
    p1 = tree.keyToCoord(groups[0].members[i1]);
    p2 = tree.keyToCoord(groups[0].members[i2]);
    p3 = tree.keyToCoord(groups[0].members[i3]);
    // Check if colinear
    if (p1.cross(p2).dot(p3) < 0.002) continue;
    
    // Get the parameters of the plane constructed from three points
    // use ax + by + cz + d = 0, where <a,b,c> is a normal of the plane
    abc = (p1-p2).cross(p1-p3);
    d = (float) -(abc.dot(p1));
    
    // Check the # of inliers
    // Reset cache
    n_inliers = 0;
    vector<OcTreeKey>().swap(inliersKey);
    for (int j=0; j<n_points; j++) {
      if (j==i1 || j==i2 || j==i3) continue;
      point3d cur_point = tree.keyToCoord(groups[0].members[j]);
      // printf("error looks like %f\n", fabs(cur_point.dot(abc)+d));
      if (fabs(cur_point.dot(abc)+d) < eps) {
        n_inliers++;
        inliersKey.push_back(groups[0].members[j]);
      }
    }
    if (n_inliers>most_inliers) {
      most_inliers = n_inliers;
      bestInliersKey = inliersKey;
    }
    // TODO: we can early terminate the iteration if...
  } // End of interation
  
  // Use SVD to get the final parameters
  printf("%d inliers out of %d\n", most_inliers, n_points);
  
  return 0;
}
