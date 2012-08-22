#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <math.h>
#include <vector>
#include "Lut.h"
#include "Transform.h"
#include "fk_tables.h"

// Kinematics.h : kinematics interface for actuator teststand 
///////////////////////////////////////////////////////////////////////////

const double PI = 2*asin(1);
const double SQRT2 = sqrt(2);

const double home_offset[1] = {
  0.3
};
const double links[1][3] = {
  {0.3, 0, 0}
};
const double actuator_top[1][3] = {
  {0.00, 0.00, 0.08}
};
const double actuator_bot[1][3] = {
  {0.00, 0.00, 0.08}
};

// Define lookup tables for forward kinematics 
const Lut1d fk_l_knee_pitch_lut(fk_l_knee_pitch_min, fk_l_knee_pitch_max, 
  fk_l_knee_pitch_dim, &fk_l_knee_pitch_data[0]);

double distance3d(const double *point1, const double *point2);

std::vector<double> kinematics_forward_joints(const double *r);
std::vector<double> kinematics_inverse_joints(const double *q);

#endif
