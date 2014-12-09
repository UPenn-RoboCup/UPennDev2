#include "normal_clustering.h"

using namespace std;
using namespace octomap;

static vector<group> groups;

void associate(point3d new_normal, OcTreeKey key, int max_n_group) {
  double min_da = 4.00;
  int ind = -1;
  // Look for the closest group
  for (int i=0; i<groups.size(); i++) {
    double da = new_normal.angleTo(groups[i].mean_normal);
    if (da<min_da) {
      ind = i;
      min_da = da;
    }
  }
  // printf("min_da: %f \n", min_da);
  // If doesn't belong to any existing group
  if (ind==-1 || min_da>(double)DEG2RAD(30)) {
    if (groups.size()>=max_n_group) {
      // printf("Discarding nodes...\n");
      return;
    }
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
    //TODO: will this overflow?
    groups[ind].sum_normal += new_normal;
    // update the mean normal  
    groups[ind].mean_normal = groups[ind].sum_normal * (1/(float)groups[ind].count);
    
    /* Printings for debugging
    printf("count: %i", groups[ind].count);
    printf("sum: %.3f %.3f %.3f\n", 
      groups[ind].sum_normal.x(),
      groups[ind].sum_normal.y(),
      groups[ind].sum_normal.z());
      
    printf("raw mean: %.3f %.3f %.3f\n", 
      groups[ind].mean_normal.x(),
      groups[ind].mean_normal.y(),
      groups[ind].mean_normal.z());
    */
      
    groups[ind].mean_normal.normalize();
  }
}


group *normal_clustering(OcTree tree, int max_num_plane) {
  groups.clear();    
  int count = 0;  
  
  for(OcTree::leaf_iterator it = tree.begin_leafs(),
         end = tree.end_leafs(); it!= end; ++it) {
      
    vector<point3d> normals;
    tree.getNormals(it.getCoordinate(), normals);
    if (normals.size()>0) {
      count++;
      associate(normals[0], it.getKey(), max_num_plane);
    }    
  }
  printf("%d leaves have normal\n", count);  
  printf("%d groups \n", groups.size());
  
  for (int g=0; g<groups.size(); g++) {
    printf("%i points, w/ normal: %.2f %.2f %.2f\n",
      groups[g].count,
      groups[g].mean_normal.x(),
      groups[g].mean_normal.y(),
      groups[g].mean_normal.z());
  }
  
  //TODO: sort the groups according to size
  
  return &groups[0];
}
