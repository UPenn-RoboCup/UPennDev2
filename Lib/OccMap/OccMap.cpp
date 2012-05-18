// Class for Robot Local Occupancy Map

#include <OccMap.h>
#include <cassert>
#include <ctime>
#include <iostream>

OccMap::OccMap()
:map_size(50)
,map_size_metric(1)
,resolution(map_size / map_size_metric)
,grid_num(map_size * map_size)
,rx(map_size / 2)
,ry(map_size * 4 / 5)
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

size_t& OccMap::get_robot_pos_x(void) {
  return rx;
}

size_t& OccMap::get_robot_pos_y(void) {
  return ry;
}
