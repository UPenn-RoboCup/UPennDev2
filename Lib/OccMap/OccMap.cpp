// Class for Robot Local Occupancy Map

#include <OccMap.h>
#include <cassert>
#include <ctime>
#include <iostream>
#include <cmath>

OccMap::OccMap()
:map_size(50)
,map_size_metric(1.0)
,resolution(map_size_metric / map_size)
,grid_num(map_size * map_size)
,rx(map_size / 2)
,ry(map_size * 4 / 5)
,max_dis(sqrt(rx * rx + ry * ry))
// initiate accumulated robot odom change
,odom_x(0.0)
,odom_y(0.0)
,odom_a(0.0)
{
  grid.resize(grid_num);
  randomize_map();
}

OccMap::~OccMap() {

}

int OccMap::randomize_map(void) {
  assert(grid.size() == grid_num);
  size_t grid_size = grid_num;
  srand(time(NULL));
  for (size_t i = 0; i < grid_num; i++) {
    grid[i] = rand() * 1.0 / RAND_MAX;
  }
  return 1;
}

vector<double>& OccMap::get_map(void) {
  return grid;
}

int& OccMap::get_robot_pos_x(void) {
  return rx;
}

int& OccMap::get_robot_pos_y(void) {
  return ry;
}

int OccMap::vision_update(double *free_bound, int width) {
  //cout << free_bound[width-1] << ' ' << free_bound[2*width-1] << endl;
  return 1;
}


int OccMap::odometry_update(const double odomX, const double odomY, const double odomA) {
//  cout << "Odom Change: " << endl;
  odom_x += odomX / resolution;
  odom_y += odomY / resolution;
  odom_a += odomA;
//  cout << odom_x << ' ' << odom_y << ' ' << odom_a << endl;
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
       // cout << rx - i - odom_x << ' ' << ry - j - odom_y << endl;
        nx = ca * (i - rx - odom_x) - sa * (ry - j - odom_y);
        ny = sa * (i - rx - odom_x) + ca * (ry - j - odom_y);
        ni = round(nx + rx);
        nj = round(ry - ny);
        if ((ni >= 0) and (ni < map_size) and (nj >= 0) and (nj < map_size))
          grid[nj * map_size + ni] = grid_temp[j * map_size + i];
      //  cout << i << ' ' << j << ' ' << ni << ' ' << ' ' << nj << endl;
      }

//    cout << "Update Odometry: " << odom_x << ' ' << odom_y << ' ' << odom_a*max_dis << endl;
    odom_x = 0;
    odom_y = 0;
    odom_a = 0;
  }
  return 1;
}
