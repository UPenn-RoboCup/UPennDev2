/*
(c) 2014 Stephen G. McGill
Kinematics for KUKA YouBot's 5 DOF arm
*/

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

const double PI_HALF = asin(1);
const double PI = 2*PI_HALF;
const double PI_ALMOST = .9*PI;
const double PI_DOUBLE = 4*PI_HALF;
const double SQRT2 = sqrt(2);

// Based on User Manual
// arm1: from center of the z rotation to the protrusion of the arm
const double baseLength = .020;
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
std::vector<double> YouBot_kinematics_inverse_arm(Transform tr, std::vector<double> q, char& is_reach_back, bool use_safe_yaw);
std::vector<double> YouBot_kinematics_inverse_arm_position(double x, double y, double z, std::vector<double> q);

#endif
