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
  const double EXT_LOG = 15;
  if (num > EXT_LOG) return EXT_LOG; // log likelihood 0.9999
  if (num < -EXT_LOG) return -EXT_LOG; // log likelihood 0.0001
  return num;
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
    grid[i] = log(rand() * 0.5 / RAND_MAX);
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
  double decay_coef = 0.001;
  double P = 0, P1 = 0;
  int i = 0;
  for (i = 0; i < grid_num; i++) {
    P = 1 - 1 / (1 + exp(grid[i]));
    P1 = P * exp(-decay_coef * (time - grid_updated_time[i]));
    grid[i] = range_check(log(P1 / (1 - P1)));
  }
  return 1;
}

int OccMap::vision_update(double *free_bound, double *free_bound_type,
    int width, double time) {
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
  var_x = 0.01;
  var_y = 0.02;
  double ob_x = 0, ob_y = 0;
  // Calculate coef for every gaussian
  for (int i = 0; i < width; i++) {
    ob_x = free_bound[i];
    ob_y = free_bound[i + width];
    gau_theta[i] = atan2(ob_x, ob_y); 
    gau_a[i] = 0.5 * pow(cos(gau_theta[i]) / var_x, 2) + 
                0.5 * pow(sin(gau_theta[i]) / var_y, 2);
    gau_b[i] = -0.25 * sin(2 * gau_theta[i]) / pow(var_x, 2) +
                0.25 * sin(2 * gau_theta[i]) / pow(var_y, 2);
    gau_c[i] = 0.5 * pow(sin(gau_theta[i]) / var_x, 2) +
                0.5 * pow(cos(gau_theta[i]) / var_y, 2);
  }
  // Update grid
  double grid_p = 0, grid_p_max = 0; // likelihood of one point on grid
  double x_robot = 0, y_robot = 0; // grid point with metric position
  for (int i = 0; i < map_size; i++)
    for (int j = 0; j < map_size; j++) {
      // (1, 2) -> (0.02, 0.04) -> (0.74, 0.48) on robot coordinate
      x_robot = ry * resolution - j * resolution;
      y_robot = rx * resolution - i * resolution;
      grid_p_max = 0;
      // Go through all observation to decide the max likelihood
      for (int k = 0; k < width; k++) {
        ob_x = free_bound[k];
        ob_y = free_bound[k + width];
        if ((int)free_bound_type[k] == 2) {
          grid_p = 0;
        } else
          grid_p = exp(- (gau_a[k] * (x_robot - ob_x) * (x_robot - ob_x) + 
                    2 * gau_b[k] * (x_robot - ob_x) * (y_robot - ob_y) + 
                        gau_c[k] * (y_robot - ob_y) * (y_robot - ob_y)));
        grid_p_max = max(grid_p_max, grid_p);
      }
      // update log likelihood -- set 0.0001 as 0
      if (grid_p_max > 0.0001) {
        double obser_p = log(grid_p_max / (1 - grid_p_max));
        grid[j * map_size + i] += obser_p;
        grid[j * map_size + i] -= log(0.25 / 0.75);
        grid[j * map_size + i] = range_check(grid[j * map_size + i]);
        grid_updated_time[j * map_size + i] = time;
      }
    }
//  cout << "likelihood update" << endl;
  return 1;
}


int OccMap::odometry_update(const double odomX, const double odomY,
    const double odomA) {
  odom_x += odomX;
  odom_y += odomY;
  odom_a += odomA;
  if ((odom_x > resolution) || (odom_y > resolution) || 
      (odom_a * max_dis > resolution)) {
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
        nx = ca * (ry - j * resolution - odom_x) - 
              sa * (rx - i * resolution - odom_y);
        ny = sa * (ry - j * resolution - odom_x) + 
              ca * (rx - i * resolution - odom_y);
        ni = round((rx - ny) / resolution);
        nj = round((ry - nx) / resolution);
        if ((ni >= 0) and (ni < map_size) and (nj >= 0) and (nj < map_size))
          grid[nj * map_size + ni] = grid_temp[j * map_size + i];
      }

    odom_x = 0;
    odom_y = 0;
    odom_a = 0;
  }
  return 1;
}
