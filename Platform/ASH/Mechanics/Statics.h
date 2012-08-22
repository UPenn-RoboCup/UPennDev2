#ifndef STATICS_H
#define STATICS_H

#include <math.h>
#include <vector>
#include "Transform.h"


const double axes [12][3] = {
  {0, 0, 1}, //begin left leg
  {1, 0, 0},
  {0, 1, 0},
  {0, 1, 0},
  {0, 1, 0},
  {1, 0, 0},
  {0, 0, 1}, //begin right leg
  {1, 0, 0},
  {0, 1, 0},
  {0, 1, 0},
  {0, 1, 0},
  {1, 0, 0}
};  


double dot_product(const double *v, const double *w);
std::vector<double> find_unit_torque ( Transform t, int act_index, int link_index, double r, double v);
std::vector<double> cross_product(const double *v, const double *w);
std::vector<double> scalar_mult( double s, const double *v);
std::vector<double> add_vectors(const double *v, const double *w);
std::vector<double> subtract_vectors(const double *v1, const double *v2);
void matrix_inverse(double invert[][2]);
void generate_FOT(double fot[][2], const double *t1, const double *t2, double axes1[], double axes2[]);
std::vector<double> statics_forward_joints(const double *v, const double *q, const double *r);
std::vector<double> statics_inverse_joints(const double *t, const double *q, const double *r);

#endif
