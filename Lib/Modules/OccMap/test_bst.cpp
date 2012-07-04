#include <iostream>
#include <vector>
#include <algorithm>

using namespace std;

struct grid_point {
  int key;
  double value;
};

bool grid_compare(struct grid_point P1, struct grid_point P2) {
  return P1.key < P2.key;
}

int main() {
  struct grid_point grid_pt[] = {{4,3.45},{2,4.089},{3,3.564},{5,1.204},{8,9.504}};
  vector<grid_point> pt_vector(grid_pt, grid_pt + 5);
  sort(pt_vector.begin(), pt_vector.end(), grid_compare);

  struct grid_point new_pt = {2,5.63};

  vector<grid_point>::iterator low;
  low = lower_bound(pt_vector.begin(), pt_vector.end(), new_pt, grid_compare);
  if (low == pt_vector.end() || low->key != new_pt.key) {
    cout << "add new pt" << endl;
    pt_vector.push_back(new_pt);
    sort(pt_vector.begin(), pt_vector.end(), grid_compare);
  }
  else {
    cout << "update old pt" << endl;
    low->value = max(low->value, new_pt.value);
  }

  vector<grid_point>::iterator it;
  for (it = pt_vector.begin(); it != pt_vector.end(); ++it)
    cout << ' ' << it->key << ' ' << it->value << endl;
  return 0;
}
