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
  int reset_size(int map_size, int robot_x, int robot_y);
  int get_map(vector<double>& map);
  int& get_robot_pos_x(void);
  int& get_robot_pos_y(void);
  
  int odometry_update(const double odomX, const double odomY, const double odomA);
  int vision_update(double *free_bound, int width);

private:
  // Map size in grids
  int map_size;
  // Map actual size
  double map_size_metric;
  // Grid num
  int grid_num;
  // Map Max resolution
  double resolution;
  // Map Structure
  vector<double> grid;

  // Robot Position in Grid
  int rx;
  int ry;
  // distance from Robot position to origin
  double max_dis;
  // Accumulated Robot Odometry Change
  double odom_x;
  double odom_y;
  double odom_a;

};

#endif
