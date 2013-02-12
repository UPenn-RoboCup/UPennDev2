#ifndef THOROP_KINEMATICS_H_
#define THOROP_KINEMATICS_H_

#include <math.h>
#include <vector>
#include "Transform.h"

const double PI = 2*asin(1);
const double SQRT2 = sqrt(2);

const double neckOffsetZ = 0;
const double neckOffsetX = 0;

//COM assumed at the chestYaw joint
const double shoulderOffsetX = 0;
const double shoulderOffsetY = .219;
const double shoulderOffsetZ = .144;

const double upperArmLength = .246;
const double elbowOffsetX = .030; //Elbow offset
const double lowerArmLength = .242;
const double handOffsetX = 0;//TBD
const double handOffsetZ = 0;//TBD

const double hipOffsetX = 0.01;
const double hipOffsetY = 0.094;
const double hipOffsetZ = 0.372;

const double thighLength = 0.379;
const double tibiaLength = 0.380;
const double footHeight = 0.04869;//calculated from webots model
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

Transform THOROP_kinematics_forward_head(const double *q);
Transform THOROP_kinematics_forward_l_arm(const double *q);
Transform THOROP_kinematics_forward_r_arm(const double *q);
Transform THOROP_kinematics_forward_l_leg(const double *q);
Transform THOROP_kinematics_forward_r_leg(const double *q);
std::vector<double> THOROP_kinematics_forward_joints(const double *r);

std::vector<double> THOROP_kinematics_inverse_r_arm(const Transform trArm);
std::vector<double> THOROP_kinematics_inverse_l_arm(const Transform trArm);
std::vector<double> THOROP_kinematics_inverse_r_leg(const Transform trLeg);
std::vector<double> THOROP_kinematics_inverse_l_leg(const Transform trLeg);
std::vector<double> THOROP_kinematics_inverse_joints(const double *q);

#endif
