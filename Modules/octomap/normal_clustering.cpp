#include <algorithm>
#include <math.h>
#include "normal_clustering.h"

using namespace std;
using namespace octomap;

vector<group> groups;

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

void normal_clustering(vector<group> &planes, OcTree tree, int max_num_plane) {
  int count = 0;  
  
  for(OcTree::leaf_iterator it = tree.begin_leafs(),
         end = tree.end_leafs(); it!= end; ++it) {
      
    vector<point3d> normals;
    tree.getNormals(it.getCoordinate(), normals);
    if (normals.size()>0) {
      count++;
      // We only consider up to 10 groups
      associate(normals[0], it.getKey(), 10);
    }    
  }
  printf("%d leaves have normal\n", count);  
  printf("%d groups \n", groups.size());
  
  vector<int> count_vec;
  for (int g=0; g<groups.size(); g++) {
    count_vec.push_back(groups[g].count);
  }  
  // Sort the groups according to size
  sort(count_vec.begin(), count_vec.end(), greater<int>());
  int num_plane = min((int)groups.size(), max_num_plane);
  
  printf("We want %d planes!\n", num_plane);

  // static vector<group> planes;
  // clear and shrink capacity
  //http://stackoverflow.com/questions/319292/changing-the-reserve-memory-of-c-vector
  // vector<group>().swap(planes);
  
  for (int i=0; i<num_plane; i++) {
    bool found = false;
    int j = 0;
    while (!found) {
      if (groups[j].count==count_vec[i]) {
        planes.push_back(groups[j]);
        found = true;
      }
      j++;
    }
  }
    
  for (int i=0; i<planes.size(); i++) {
    printf("%i points, mean normal: %.2f %.2f %.2f\n",
      planes[i].count,
      planes[i].mean_normal.x(),
      planes[i].mean_normal.y(),
      planes[i].mean_normal.z());
  }
  
  // return planes;

}
