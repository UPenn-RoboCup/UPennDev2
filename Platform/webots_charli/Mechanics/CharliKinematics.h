#ifndef CHARLI_KINEMATICS_H_
#define CHARLI_KINEMATICS_H_

#include "Transform.h"
#include <math.h>
#include <vector>
#include <stdio.h>

const double PI = 2*asin(1);
const double SQRT2 = sqrt(2);

const double neckOffsetZ = 0;
const double neckOffsetX = 0;
const double shoulderOffsetX = 0;
const double shoulderOffsetY = 0;
const double shoulderOffsetZ = 0;
const double handOffsetX = 0;
const double handOffsetZ = 0;
const double upperArmLength = 0;
const double lowerArmLength = 0;
const double hipOffsetY = 0.070;
const double hipOffsetZ = 0.0;
const double hipOffsetX = 0.0;
const double thighLength = 0.350;
const double tibiaLength = 0.350;
const double footHeight = 0.076;
const double kneeOffsetX = 0.0;
const double dThigh = sqrt(thighLength*thighLength+kneeOffsetX*kneeOffsetX);
const double aThigh = atan(kneeOffsetX/thighLength);
const double dTibia = sqrt(tibiaLength*tibiaLength+kneeOffsetX*kneeOffsetX);
const double aTibia = atan(kneeOffsetX/tibiaLength);

const double servoOffset[] = {
  0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
  0.0, 0.0, 0.0, 0.0, 0.0, 0.0
};

void printTransform(Transform tr);
void printVector(std::vector<double> v);

Transform charli_kinematics_forward_head(const double *q);
Transform charli_kinematics_forward_l_arm(const double *q);
Transform charli_kinematics_forward_r_arm(const double *q);
Transform charli_kinematics_forward_l_leg(const double *q);
Transform charli_kinematics_forward_r_leg(const double *q);
std::vector<double> charli_kinematics_forward_joints(const double *r);

std::vector<double> charli_kinematics_inverse_r_arm(const Transform trArm);
std::vector<double> charli_kinematics_inverse_l_arm(const Transform trArm);
std::vector<double> charli_kinematics_inverse_r_leg(const Transform trLeg);
std::vector<double> charli_kinematics_inverse_l_leg(const Transform trLeg);
std::vector<double> charli_kinematics_inverse_joints(const double *q);

#endif
