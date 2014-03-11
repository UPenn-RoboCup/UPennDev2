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
const double armLength = lowerArmLength + upperArmLength + gripperLength;

// Center of mass calculation
const double mLowerArm = 1;
const double mUpperArm = 1;
const double mWrist = .2;
const double mHand = .3;
const double mBase = 5;
const double mArm = mLowerArm + mUpperArm + mWrist + mHand;
const double mTotal = mBase + mArm;
//
const double comLowerArm[3] = {0,0,0.075};
const double comUpperArm[3] = {0,0,0.07};
const double comWrist[3]    = {0,0,.04};
const double comHand[3]     = {0,0,.05};
// How far offset from the base the arm is
const double comArm[3]      = {0.4,0,0};

Transform YouBot_kinematics_forward_arm(const double *q, char& is_singular);
std::vector<double> YouBot_kinematics_com_arm(const double *q, std::vector<double>& comObject, const double mObject);
std::vector<double> YouBot_kinematics_inverse_arm(Transform tr, std::vector<double>& q, char& is_reach_back, bool use_safe_yaw);
std::vector<double> YouBot_kinematics_inverse_arm_position(std::vector<double>& position, std::vector<double>& q, char& is_reach_back);

#endif
