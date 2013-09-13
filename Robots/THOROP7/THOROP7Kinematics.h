#ifndef THOROP7_KINEMATICS_H_
#define THOROP7_KINEMATICS_H_

#include "Transform.h"
#include <stdio.h>
#include <math.h>
#include <vector>

const double PI = 2*asin(1);
const double SQRT2 = sqrt(2);

//From COM to neck joint
const double neckOffsetZ = .144+0.027+0.114;//from webots value
const double neckOffsetX = 0.023;//from webots value

//COM assumed at the chestYaw joint
//const double shoulderOffsetX = 0;
const double shoulderOffsetX = 0;//From webots value
//const double shoulderOffsetY = .219;

const double shoulderOffsetY = .233;  //from actual robot
const double shoulderOffsetZ = .144;

const double upperArmLength = .246;
const double elbowOffsetX = .030; //Elbow offset
const double lowerArmLength = .190; //From actual 7DOF robot

//Longer wrist for 7DOF hand

const double handOffsetX = 0.245; //From actual 7DOF robot
const double handOffsetY = 0.030; //From actual 7DOF robot
const double handOffsetZ = 0;






//const double hipOffsetX = 0.01;
const double hipOffsetX = -0.001;//From webots model
const double hipOffsetY = 0.094;
//const double hipOffsetZ = 0.372;
const double hipOffsetZ = 0.384;//calculated from webots model

const double thighLength = 0.379;
const double tibiaLength = 0.380;
const double footHeight = 0.04869;//calculated from webots model
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

Transform THOROP7_kinematics_forward_head(const double *q);
Transform THOROP7_kinematics_forward_l_arm(const double *q);
Transform THOROP7_kinematics_forward_r_arm(const double *q);
Transform THOROP7_kinematics_forward_l_leg(const double *q);
Transform THOROP7_kinematics_forward_r_leg(const double *q);
std::vector<double> THOROP7_kinematics_forward_joints(const double *r);


Transform THOROP7_kinematics_forward_l_arm_7(const double *q);
Transform THOROP7_kinematics_forward_r_arm_7(const double *q);

std::vector<double> THOROP7_kinematics_inverse_r_arm(const Transform trArm, const double *qOrg);
std::vector<double> THOROP7_kinematics_inverse_l_arm(const Transform trArm, const double *qOrg);

std::vector<double> THOROP7_kinematics_inverse_r_wrist(const Transform trArm, double shoulderYaw);
std::vector<double> THOROP7_kinematics_inverse_l_wrist(const Transform trArm, double ShoulderYaw);

std::vector<double> THOROP7_kinematics_inverse_r_arm_7(const Transform trArm, double shoulderYaw);
std::vector<double> THOROP7_kinematics_inverse_l_arm_7(const Transform trArm,double shoulderYaw);

std::vector<double> THOROP7_kinematics_inverse_r_leg(const Transform trLeg);
std::vector<double> THOROP7_kinematics_inverse_l_leg(const Transform trLeg);
std::vector<double> THOROP7_kinematics_inverse_joints(const double *q);

#endif
