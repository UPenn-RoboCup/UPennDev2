#ifndef YOUBOT_KINEMATICS_H_
#define YOUBOT_KINEMATICS_H_

#include "Transform.h"
#include <stdio.h>
#include <math.h>
#include <vector>

// For pushing/pulling torch objects
#ifdef TORCH

#ifdef __cplusplus
extern "C"
{
#endif
#include <torch/luaT.h>
#include <torch/TH/TH.h>
#ifdef __cplusplus
}
#endif

#endif

const double PI = 2*asin(1);
const double SQRT2 = sqrt(2);

// Based on User Manual
// arm1: from ground to first pitch joint
const double baseLength = .155;
// arm2: 
const double lowerArmLength = .155;
// arm3: 
const double upperArmLength = .135;
// arm4: 
const double wristLength = .081;
// arm5: 
const double handLength = .105;
// helper
const double gripperLength = wristLength + handLength;

void printTransform(Transform tr);
void printVector(std::vector<double> v);

Transform YouBot_kinematics_forward_arm(const double *q);

#ifdef TORCH
std::vector<double> YouBot_kinematics_inverse_arm(const THDoubleTensor * tr);
#else
//std::vector<double> YouBot_kinematics_inverse_arm(Transform trArm);
std::vector<double> YouBot_kinematics_inverse_arm(const double *r);
#endif

#endif
