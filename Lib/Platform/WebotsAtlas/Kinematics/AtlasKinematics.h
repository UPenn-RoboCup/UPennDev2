#ifndef Atlas_KINEMATICS_H_
#define Atlas_KINEMATICS_H_

#include <math.h>
#include <vector>
#include "Transform.h"

const double PI = 2*asin(1);
const double SQRT2 = sqrt(2);

//Atlas Kinematics: based on DRCSim model
//COM position: center of pelvis + 265mm
//265mm offset to match THOR-OP


//From COM to neck joint
const double neckOffsetX = 0.165;
const double neckOffsetZ = .090+.050+.289+.79 - .265;

//From COM to 1st shoulder joint
const double shoulderOffsetX = .024;
const double shoulderOffsetY = .221;
const double shoulderOffsetZ = .090+.050+.289 - .265;
const double shoulderAngle = 30*PI/180;

//From 1st shoulder joint to 2nd shoulder joint
const double  scapulaOffsetY = .075;
const double  scapulaOffsetZ = .036;

const double upperArmLength = .306;
const double elbowOffsetX = .013;
const double lowerArmLength = .246;

//Hand offset is based on current webots model
const double handOffsetX = 0.150;
const double handOffsetY = 0.0;
const double handOffsetZ = 0;

const double hipOffsetX = -0.050;
const double hipOffsetY = 0.089;
const double hipOffsetZ = 0.050 + 0.265; //265mm offset to match THOR-OP

const double thighLength = 0.374;
const double tibiaLength = 0.422;
const double footHeight = 0.080;//calculated from webots model
const double kneeOffsetX = 0.0;

const double dThigh = sqrt(thighLength*thighLength+kneeOffsetX*kneeOffsetX);
const double aThigh = atan(kneeOffsetX/thighLength);
const double dTibia = sqrt(tibiaLength*tibiaLength+kneeOffsetX*kneeOffsetX);
const double aTibia = atan(kneeOffsetX/tibiaLength);

const double dUpperArm = sqrt(upperArmLength*upperArmLength+elbowOffsetX*elbowOffsetX);
const double dLowerArm = sqrt(lowerArmLength*lowerArmLength+elbowOffsetX*elbowOffsetX);
const double aUpperArm = atan(elbowOffsetX/upperArmLength);
const double aLowerArm = atan(elbowOffsetX/lowerArmLength);



const double servoOffset[] = {
  0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
  0.0, 0.0, 0.0, 0.0, 0.0, 0.0
};

void printTransform(Transform tr);
void printVector(std::vector<double> v);

Transform Atlas_kinematics_forward_head(const double *q);
Transform Atlas_kinematics_forward_l_arm(const double *q);
Transform Atlas_kinematics_forward_r_arm(const double *q);
Transform Atlas_kinematics_forward_l_leg(const double *q);
Transform Atlas_kinematics_forward_r_leg(const double *q);
std::vector<double> Atlas_kinematics_forward_joints(const double *r);

std::vector<double> Atlas_kinematics_inverse_r_arm(const Transform trArm);
std::vector<double> Atlas_kinematics_inverse_l_arm(const Transform trArm);

std::vector<double> Atlas_kinematics_inverse_r_wrist(const Transform trArm, double shoulderYaw);
std::vector<double> Atlas_kinematics_inverse_l_wrist(const Transform trArm, double ShoulderYaw);


std::vector<double> Atlas_kinematics_inverse_r_leg(const Transform trLeg);
std::vector<double> Atlas_kinematics_inverse_l_leg(const Transform trLeg);
std::vector<double> Atlas_kinematics_inverse_joints(const double *q);

#endif
