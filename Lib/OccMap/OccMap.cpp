// Class for Robot Local Occupancy Map

#include "OccMap.h"
#include <cassert>
#include <time.h>
#include <stdio.h>
#include <iostream>
#include <iomanip>
#include <cmath>
#include <algorithm>


OccMap::OccMap()
:map_size(50), map_size_metric(1.0) ,resolution(map_size_metric / map_size)
,grid_num(map_size * map_size)
,rx(25), ry(40)
,max_dis(sqrt(rx * rx + ry * ry))
// initiate accumulated robot odom change
,odom_x(0.0), odom_y(0.0), odom_a(0.0)
,odom_change_num(0)
// vars for vision update
,var_x(0.0), var_y(0.0)
,gau_max_range(0.0)
,gau_min_p(0.0001)
,default_p(0.25)
,default_log_p(log(default_p / (1 - default_p)))
{
}

inline void OccMap::range_check(double &num) {
  const double EXT_LOG = 15;
  if (num > EXT_LOG) num =  EXT_LOG; // log likelihood 0.9999
  if (num < -EXT_LOG) num = -EXT_LOG; // log likelihood 0.0001
}

int OccMap::reset_size(int size, int robot_x, int robot_y, double time) {
  map_size = size;
  resolution = map_size_metric / map_size;
  grid_num = map_size * map_size;
  rx = robot_x;
  ry = robot_y;
  max_dis = sqrt(rx * rx + ry * ry);
  if (grid.size() < grid_num) {
    grid_out.resize(grid_num);
    grid.resize(grid_num);
  }
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
    grid[i] = log(rand() * default_p / RAND_MAX);
  }
  return 1;
}

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

int OccMap::time_decay(double time) {
  double decay_coef = 0.001;
  double P = 0, P1 = 0;
  int i = 0;
  for (i = 0; i < grid_num; i++) {
    P = 1 - 1 / (1 + exp(grid[i]));
    P1 = P * exp(-decay_coef * (time - grid_updated_time[i]));
    grid[i] = log(P1 / (1 - P1));
    range_check(grid[i]);
  }
  return 1;
}

int OccMap::vision_proc_init(int obs_width) {
  cout << "Occupancy Map Vision Proc Initiation" << endl;
  if (gau_a.size() < obs_width) {
      gau_a.resize(obs_width);  
      cout << "resize container for Gaussians Coef a" << endl;
  } 
  if (gau_b.size() < obs_width) {
      gau_b.resize(obs_width);  
      cout << "resize container for Gaussians Coef b" << endl;
  }
  if (gau_c.size() < obs_width) {
      gau_c.resize(obs_width);  
      cout << "resize container for Gaussians Coef c" << endl;
  }
  if (gau_theta.size() < obs_width) {
      gau_theta.resize(obs_width);  
      cout << "resize container for Gaussians Rotation Angles" << endl;
  }
  
  var_x = 0.01;
  cout << "initiate x variance " << var_x << endl;
  var_y = 0.02;
  cout << "initiate y variance " << var_y << endl;
  
  gau_max_range = sqrt(-log(gau_min_p) * 2 * pow(min(var_x, var_y), 2));
  cout << "initiate gaussian max effective range " << gau_max_range << endl;

  cout << "initiate default likelihood for unknown area " << default_p << endl;
  cout << "initiate default log likelihood for unknown area " << default_log_p << endl;
  return 1;
}

int OccMap::vision_update(vector<double>& free_bound, vector<int>& free_bound_type,
    int width, double time) {
  clock_t start = clock();
  /*
  for (int i = 0; i < free_bound.size(); i++)
    cout << free_bound[i] << ' ';
  cout << endl;
  for (int i = 0; i < free_bound_type.size(); i++)
    cout << free_bound_type[i] << ' ';
  cout << endl; 
  */
  double ob_x = 0, ob_y = 0;
  // Calculate coef for every gaussian
  for (int i = 0; i < width; i++) {
    gau_theta[i] = atan2(free_bound[i], free_bound[i + width]); 
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
        if (free_bound_type[k] == 2) {
          grid_p = 0;
        } else
          grid_p = exp(- (gau_a[k] * (x_robot - ob_x) * (x_robot - ob_x) + 
                    2 * gau_b[k] * (x_robot - ob_x) * (y_robot - ob_y) + 
                        gau_c[k] * (y_robot - ob_y) * (y_robot - ob_y)));
        grid_p_max = max(grid_p_max, grid_p);
      }
      // update log likelihood -- set 0.0001 as 0
      if (grid_p_max > gau_min_p) {
        grid[j * map_size + i] += log(grid_p_max / (1 - grid_p_max));
        grid[j * map_size + i] -= default_log_p;
        range_check(grid[j * map_size + i]);
        grid_updated_time[j * map_size + i] = time;
      }
    }
  clock_t stop = clock();
  cout << "Vision Update Finished in " << (double)(stop - start)/CLOCKS_PER_SEC << " seconds." << endl;
  return 1;
}

int OccMap::odometry_init(void) {
  odom_change.resize(grid_num);
  odom_change_num = 0;
  return 1;
}

int OccMap::odometry_update(const double odomX, const double odomY,
    const double odomA) {
  odom_x += odomX;
  odom_y += odomY;
  odom_a += odomA;
//  cout << odom_x << ' ' << odom_y << ' ' << odom_a << endl;
  if ((odom_x > resolution) || (odom_y > resolution) || 
      (odom_a * max_dis > resolution)) {
//    cout << "odom update" << endl;
    double nx = 0; // new point in occmap coordinate x
    double ny = 0; // new point in occmap coordinate y
    int ni = 0; // new point on map x;
    int nj = 0; // new point on map y;
    double ca = cos(odom_a);
    double sa = sin(odom_a);
    odom_change_num = 0;
    for (int i = 0; i < map_size; i++)
      for (int j = 0; j < map_size; j++) {
        nx = ca * (ry - j * resolution - odom_x) - sa * (rx - i * resolution - odom_y);
        ny = sa * (ry - j * resolution - odom_x) + ca * (rx - i * resolution - odom_y);
        ni = round((rx - ny) / resolution);
        nj = round((ry - nx) / resolution);
        if ((ni >= 0) && (ni < map_size) && (nj >= 0) && (nj < map_size)) 
          continue;
        if ((i == ni) || (j == nj)) 
          continue;
        odom_change[odom_change_num].x = ni;
        odom_change[odom_change_num].y = nj;
        odom_change[odom_change_num].value = grid[j * map_size + i];
        odom_change_num++;
      }
//    cout << odom_change_num << " blocks updated" << endl; 
    for (int cnt = 0; cnt < odom_change_num; cnt ++) {
      ni = odom_change[cnt].x;
      nj = odom_change[cnt].y;
      grid[nj * resolution + ni] = odom_change[cnt].value;
    }

    odom_x = 0;
    odom_y = 0;
    odom_a = 0;
  }
  return 1;
}
