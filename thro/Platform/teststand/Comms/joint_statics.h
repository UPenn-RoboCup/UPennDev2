#ifndef _JOINT_STATICS_H_
#define _JOINT_STATICS_H_

#include <math.h>
#include <vector>
#include "Transform.h"

// joint_statics.h : statics interface for actuator teststand 
///////////////////////////////////////////////////////////////////////////

const double axes[1][3] = {
  {0, 1, 0}
};

std::vector<double> statics_forward_joints(const double *v, const double *q, const double *r);
std::vector<double> statics_inverse_joints(const double *t, const double *q, const double *r);

#endif
