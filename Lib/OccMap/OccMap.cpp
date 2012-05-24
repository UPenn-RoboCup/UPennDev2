// Class for Robot Local Occupancy Map

#include "OccMap.h"
#include <cassert>
#include <ctime>
#include <iostream>
#include <iomanip>
#include <cmath>
#include <algorithm>


OccMap::OccMap()
:map_size(50)
,map_size_metric(1.0)
,resolution(map_size_metric / map_size)
,grid_num(map_size * map_size)
,rx(25)
,ry(40)
,max_dis(sqrt(rx * rx + ry * ry))
// initiate accumulated robot odom change
,odom_x(0.0)
,odom_y(0.0)
,odom_a(0.0)

,var_x(0.0)
,var_y(0.0)
{
}

OccMap::~OccMap() {

}

inline double OccMap::range_check(double num) {
  if (num > 10) return 10; // log likelihood 0.9999
  if (num < -10) return -10; // log likelihood 0.0001
}

int OccMap::reset_size(int size, int robot_x, int robot_y, double time) {
  map_size = size;
  resolution = map_size_metric / map_size;
  grid_num = map_size * map_size;
  rx = robot_x;
  ry = robot_y;
  max_dis = sqrt(rx * rx + ry * ry);
  if (grid.size() < grid_num) 
    grid.resize(grid_num);
  randomize_map();

  // reset map init time
  if (grid_updated_time.size() < grid_num)
    grid_updated_time.resize(grid_num);
  for (int i = 0; i < grid_num; i++)
    grid_updated_time[i] = time;
  return 1;
}

int OccMap::randomize_map(void) {
  assert(grid.size() == grid_num);
  int grid_size = grid_num;
  srand(time(NULL));
  for (int i = 0; i < grid_num; i++) {
    grid[i] = log(rand() * 1.0 / RAND_MAX);
  }
  return 1;
}

int OccMap::get_map(vector<double> & map) {
  for (int i = 0; i < grid.size(); i++)
    map.push_back(grid[i]);
  return 1;
}

int OccMap::get_map_updated_time(vector<double> & updated_time) {
  for (int i = 0; i < grid_updated_time.size(); i++)
    updated_time.push_back(grid_updated_time[i]);
  return 1;
}

int& OccMap::get_robot_pos_x(void) {
  return rx;
}

int& OccMap::get_robot_pos_y(void) {
  return ry;
}

int OccMap::time_decay(double time) {
  double decay_coef = 0.5;
  for (int i = 0; i < grid_num; i++) {
    grid[i] = range_check(exp(-decay_coef * 
                            (time - grid_updated_time[i])) *grid[i]);
  }
  return 1;
}

int OccMap::vision_update(double *free_bound, double *free_bound_type,
    int width, double time) {
  cout << setprecision(15) << "last_updated_time " << grid_updated_time[0] << 
          " last time " << time - grid_updated_time[0] << endl;
  grid_updated_time[0] = time;
/*
  if (gau_a.size() < width) {
      gau_a.resize(width);  
      cout << "resize container for Gaussians Coef a" << endl;
  } 
  if (gau_b.size() < width) {
      gau_b.resize(width);  
      cout << "resize container for Gaussians Coef b" << endl;
  }
  if (gau_c.size() < width) {
      gau_c.resize(width);  
      cout << "resize container for Gaussians Coef c" << endl;
  }
  if (gau_theta.size() < width) {
      gau_theta.resize(width);  
      cout << "resize container for Gaussians Rotation Angles" << endl;
  }
  // Suppose var constant first here
  var_x = 0.04;
  var_y = 0.04;
  double ob_x = 0, ob_y = 0;
  // Calculate coef for every gaussian
  for (int i = 0; i < width; i++) {
    ob_x = free_bound[i];
    ob_y = free_bound[i + width];
    gau_theta[i] = atan2(-ob_y, ob_x); 
    gau_a[i] = 0.5 * pow(cos(gau_theta[i]) / var_x, 2) + 
                0.5 * pow(sin(gau_theta[i]) / var_y, 2);
    gau_b[i] = -0.25 * sin(2 * gau_theta[i]) / pow(var_x, 2) +
                0.25 * sin(2 * gau_theta[i]) / pow(var_y, 2);
    gau_c[i] = 0.5 * pow(sin(gau_theta[i]) / var_x, 2) +
                0.5 * pow(cos(gau_theta[i]) / var_y, 2);
  }
  // Update grid
  double grid_p = 0, grid_p_max = 0; // likelihood of one point on grid
  double i_metric = 0, j_metric = 0; // grid point with metric position
  for (int i = 0; i < map_size; i++)
    for (int j = 0; j < map_size; j++) {
      // (1, 2) on grid -> (0.02, 0.04) on real OccMap
      i_metric = i * resolution;
      j_metric = j * resolution; 
      // Go through all observation to decide the max likelihood
      for (int k = 0; k < width; k++) {
        ob_x = free_bound[i];
        ob_y = free_bound[i + width];
        grid_p = 1 / exp(gau_a[k] * pow(i_metric - ob_x, 2) + 
                        2 * gau_b[k] * (i_metric - ob_x) * (j_metric - ob_y) + 
                        gau_c[k] * pow(j_metric - ob_y, 2));
        grid_p_max = max(grid_p_max, grid_p);
      }
      // update log likelihood
      grid[j * map_size + i] += log(grid_p / (1 - grid_p));
    }
*/
  return 1;
}


int OccMap::odometry_update(const double odomX, const double odomY,
    const double odomA) {
  odom_x += odomX / resolution;
  odom_y += odomY / resolution;
  odom_a += odomA;
  if ((odom_x > 1) || (odom_y > 1) || (odom_a * max_dis > 1)) {
    double nx = 0; // new point in occmap coordinate x
    double ny = 0; // new point in occmap coordinate y
    int ni = 0; // new point on map x;
    int nj = 0; // new point on map y;
    double ca = cos(odom_a);
    double sa = sin(odom_a);
    vector<double> grid_temp = grid;
    randomize_map();
    for (int i = 0; i < map_size; i++)
      for (int j = 0; j < map_size; j++) {
        nx = ca * (i - rx - odom_x) - sa * (ry - j - odom_y);
        ny = sa * (i - rx - odom_x) + ca * (ry - j - odom_y);
        ni = round(nx + rx);
        nj = round(ry - ny);
        if ((ni >= 0) and (ni < map_size) and (nj >= 0) and (nj < map_size))
          grid[nj * map_size + ni] = grid_temp[j * map_size + i];
      }

    odom_x = 0;
    odom_y = 0;
    odom_a = 0;
  }
  return 1;
}
