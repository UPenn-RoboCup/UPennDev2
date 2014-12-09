#include "normal_clustering.h"

using namespace std;
using namespace octomap;

static vector<group> groups;

void associate(point3d new_normal, OcTreeKey key, int max_n_group) {
  double min_da = M_PI;
  int ind = -1;
  // Look for the closest group
  for (int i=0; i<groups.size(); i++) {
    double da = new_normal.angleTo(groups[i].mean_normal);
    if (da<min_da) {
      ind = i;
      min_da = da;
    }
  }
  printf("min_da: %f \n", min_da);
  // If doesn't belong to any existing group
  if (ind==-1 || min_da>(double)DEG2RAD(30)) {
    printf("new group...\n");
    group new_group;
    new_group.count = 1;
    new_group.mean_normal = new_normal;
    new_group.sum_normal = new_normal;
    new_group.members.push_back(key);
    groups.push_back(new_group);
  } else {
    groups[ind].count += 1;
    groups[ind].members.push_back(key);
    groups[ind].sum_normal += new_normal;
    // update the mean normal  
    groups[ind].mean_normal = groups[ind].sum_normal * (1/groups[ind].count);
    groups[ind].mean_normal.normalize();
  }
}


group *normal_clustering(OcTree tree, int max_num_plane) {
  groups.clear();  
  for(OcTree::leaf_iterator it = tree.begin_leafs(),
         end = tree.end_leafs(); it!= end; ++it) {
      
    vector<point3d> normals;
    tree.getNormals(it.getCoordinate(), normals);
    if (normals.size()>0) associate(normals[0], it.getKey(), max_num_plane);

  }
  printf("%d groups \n", groups.size());
  // for (int g=0; g<groups.size(); g++) {
  //   printf("Mean normal: %.2f %.2f %.2f\n",
  //     groups[g].mean_normal.x(),
  //     groups[g].mean_normal.y(),
  //     groups[g].mean_normal.z());
  // }
  return &groups[0];
}
