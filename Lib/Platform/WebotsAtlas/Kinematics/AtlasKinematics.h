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
const double shoulderOffsetX1 = .024;
const double shoulderOffsetY1 = .221;
const double shoulderOffsetZ1 = .090+.050+.289 - .265;
const double shoulderRollAngle = 60*PI/180;

//From 1st shoulder joint to 2nd shoulder joint
const double  scapulaOffsetY = .075;
const double  scapulaOffsetZ = .036;

//Projection point of the shoulder pitch joint
const double shoulderOffsetX = shoulderOffsetX1;
const double shoulderOffsetY = shoulderOffsetY1 + .03434;
const double shoulderOffsetZ = shoulderOffsetZ1 + .05948;

//Projected shoulder joint to shoulder Roll joint
const double upperArmLength0 = .046951;

//Actual length between shoulder Roll to elbow
const double upperArmLength1 = .306;

//Approximate upper arm (exact when shoulder Roll = 0)
const double upperArmLength = upperArmLength1 + upperArmLength0;

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
const double kneeOffsetX1 = 0.050;
const double kneeOffsetX2 = 0.0;

const double dThigh = sqrt(thighLength*thighLength+kneeOffsetX1*kneeOffsetX1);
const double aThigh = atan(kneeOffsetX1/thighLength);
const double dTibia = sqrt(tibiaLength*tibiaLength+kneeOffsetX2*kneeOffsetX2);
const double aTibia = atan(kneeOffsetX2/tibiaLength);

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
