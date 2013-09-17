#ifndef THOROP6_KINEMATICS_H_
#define THOROP6_KINEMATICS_H_

#include "Transform.h"
#include <stdio.h>
#include <math.h>
#include <vector>

const double PI = 2*asin(1);
const double SQRT2 = sqrt(2);

//From COM to neck joint
#define neckOffsetZ .144+0.027+0.114//from webots value
#define neckOffsetX 0.023//from webots value

//COM assumed at the chestYaw joint
//#define shoulderOffsetX 0
#define shoulderOffsetX 0//From webots value
//#define shoulderOffsetY .219

//#define shoulderOffsetY .255  //from actual robot
#define shoulderOffsetY .259  //from actual robot
#define shoulderOffsetZ .144

#define upperArmLength .246
//#define lowerArmLength .242 //Webots value
#define lowerArmLength .215 //From actual robot
#define elbowOffsetX .030 // Elbow offset

//Hand offset is based on current webots model
//We assume hand is offseted inside
#define handOffsetX 0.12 //From actual robot
#define handOffsetY 0.05
#define handOffsetZ 0


//#define hipOffsetX 0.01
#define hipOffsetX -0.001//From webots model
#define hipOffsetY 0.094
//#define hipOffsetZ 0.372
#define hipOffsetZ 0.384//calculated from webots model

#define thighLength 0.379
#define tibiaLength 0.380
#define footHeight 0.04869//calculated from webots model
#define kneeOffsetX 0.0

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

Transform THOROP_kinematics_forward_head(const double *q);
Transform THOROP_kinematics_forward_l_arm(const double *q);
Transform THOROP_kinematics_forward_r_arm(const double *q);
Transform THOROP_kinematics_forward_l_leg(const double *q);
Transform THOROP_kinematics_forward_r_leg(const double *q);
std::vector<double> THOROP_kinematics_forward_joints(const double *r);


Transform THOROP_kinematics_forward_l_arm_7(const double *q);
Transform THOROP_kinematics_forward_r_arm_7(const double *q);

std::vector<double> THOROP_kinematics_inverse_r_arm(const Transform trArm, const double *qOrg);
std::vector<double> THOROP_kinematics_inverse_l_arm(const Transform trArm, const double *qOrg);

std::vector<double> THOROP_kinematics_inverse_r_wrist(const Transform trArm, double shoulderYaw);
std::vector<double> THOROP_kinematics_inverse_l_wrist(const Transform trArm, double ShoulderYaw);

std::vector<double> THOROP_kinematics_inverse_r_arm_7(const Transform trArm, double shoulderYaw);
std::vector<double> THOROP_kinematics_inverse_l_arm_7(const Transform trArm,double shoulderYaw);


std::vector<double> THOROP_kinematics_inverse_r_leg(const Transform trLeg);
std::vector<double> THOROP_kinematics_inverse_l_leg(const Transform trLeg);
std::vector<double> THOROP_kinematics_inverse_joints(const double *q);

#endif
