#include "OccMap.h"

void OccMap::velocity_generation(double attackBearing, double *velocity,
                                double attractScale, double repulseScale) {
  double repulsion[2] = {0.0, 0.0}; // robot coordinate x, y
  double attraction[2] = {0.0, 0.0}; // robot coordinate x, y
  int nrepulsion = 0;
  double grid_prob = 0, dist = 0;
  int i = 0, j = 0;
  double x = 0.0, y = 0.0;
  double validRange = 0.15;
  for (int cnt = 0; cnt < grid_num; cnt++) {
    if (grid[cnt] > obs_log_p) {
      grid_prob = (1 - 1/(1+exp(grid[cnt])));
      // Occupied Grid
      i = cnt % map_size;
      j = (cnt - j) / map_size;
      x = ry * resolution - j * resolution;
      y = rx * resolution - i * resolution;
      dist = sqrt((x - odom_x) * (x - odom_x) + (y - odom_y) * (y - odom_y));
      if (dist > validRange)
        continue;
      ++nrepulsion;
      repulsion[0] += (x - odom_x) / dist * grid_prob;
      repulsion[1] += (y - odom_y) / dist * grid_prob;
    }
  }
  double attractAngle = 0, repulseAngle = 0;
  attractAngle = attackBearing;
  
  if (nrepulsion > 0) {
    repulseAngle = atan2(repulsion[1], repulsion[2]) - odom_a;
  }
  else {
    repulseScale = 0;
  }

  velocity[0] = attractScale * cos(attractAngle) - repulseScale * cos(repulseAngle);
  velocity[1] = attractScale * sin(attractAngle) - repulseScale * sin(repulseAngle);
  velocity[2] = atan2(velocity[1], velocity[0]);
//  cout << attractScale << ' ' <<  attractAngle << ' ' << velocity[2] << endl;
}
