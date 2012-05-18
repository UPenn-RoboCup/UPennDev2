// Class Header for Robot Local Occupancy Map
#ifndef __OCCMAP_H__
#define __OCCMAP_H__

#include <vector>

using namespace std;

// Gird Map Coordinate
// 0 -------------> X
// |
// |
// y

class OccMap {
public:
  OccMap();
  ~OccMap();
  int randomize_map(void);
  vector<double>& get_map(void);
  size_t& get_robot_pos_x(void);
  size_t& get_robot_pos_y(void);

private:
  // Map size in grids
  size_t map_size;
  // Map actual size
  size_t map_size_metric;
  // Grid num
  size_t grid_num;
  // Map Max resolution
  double resolution;
  // Map Structure
  vector<double> grid;

  // Robot Position in Grid
  size_t rx;
  size_t ry;

};

#endif
