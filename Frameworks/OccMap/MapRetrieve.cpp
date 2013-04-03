#include "OccMap.h"

vector<uint32_t>& OccMap::get_map(void) {
  for (int i = 0; i < grid.size(); i++)
    grid_out[i] = (1 - 1/(1+exp(grid[i]))) * 10000;
  return grid_out;
}

vector<double>& OccMap::get_map_updated_time(void) {
  return grid_updated_time;
}

int& OccMap::get_robot_pos_x(void) {
  return rx;
}

int& OccMap::get_robot_pos_y(void) {
  return ry;
}

int OccMap::get_nobstacle() {
  return nOb;
}

void OccMap::get_obstacle(int index, obstacle& obs_out) {
   obs_out = obs[index];
}

int OccMap::get_odometry(double& pose_x, double& pose_y, double& pose_a) {
  pose_x = odom_x;
  pose_y = odom_y;
  pose_a = odom_a;
  return 1;
}

