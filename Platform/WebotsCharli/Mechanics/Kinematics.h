#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <math.h>
#include <vector>
#include "Transform.h"

const double PI = 2*asin(1);
const double SQRT2 = sqrt(2);

// Define constants for inverse kinematics
const double neckOffsetZ = 0;
const double neckOffsetX = 0;
const double shoulderOffsetX = 0;
const double shoulderOffsetY = 0;
const double shoulderOffsetZ = 0;
const double handOffsetX = 0;
const double handOffsetZ = 0;
const double upperArmLength = 0;
const double lowerArmLength = 0;
const double hipOffsetY = .097; //updated to SAFFiR  DFL  120218
const double hipOffsetZ = .00;
const double hipOffsetX = .000;
const double thighLength = .379025;
const double tibiaLength = .3800;
const double footHeight = .0487;
const double kneeOffsetX = .00255;
const double dThigh = sqrt(thighLength*thighLength+kneeOffsetX*kneeOffsetX);
const double aThigh = atan(kneeOffsetX/thighLength);
const double dTibia = tibiaLength; //changed to eliminate kneeOffsetX for SAFFiR
const double aTibia = 0; //changed to eliminate kneeOffsetX for SAFFiR
const double servoOffset[] = {
  0.0, 0.2954, 0.3025, 0.2566, 0.3167, 0.3166, // l_leg
  0.0, 0.2954, 0.3025, 0.2566, 0.3167, 0.3166  // r_leg
};

void printTransform(Transform tr);
void printVector(std::vector<double> v);
double distance3d(const double *point1, const double *point2);

Transform kinematics_forward_head(const double *q);
Transform kinematics_forward_l_arm(const double *q);
Transform kinematics_forward_r_arm(const double *q);
Transform kinematics_forward_l_leg(const double *q);
Transform kinematics_forward_r_leg(const double *q);

std::vector<double> kinematics_inverse_r_arm(const Transform trArm);
std::vector<double> kinematics_inverse_l_arm(const Transform trArm);
std::vector<double> kinematics_inverse_r_leg(const Transform trLeg);
std::vector<double> kinematics_inverse_l_leg(const Transform trLeg);

#endif
