#ifndef THOROP7_KINEMATICS_H_
#define THOROP7_KINEMATICS_H_

#include "Transform.h"
#include <stdio.h>
#include <math.h>
#include <vector>

const double PI = 2*asin(1);
const double SQRT2 = sqrt(2);

/*
//From COM to neck joint
const double neckOffsetZ = .144+0.027+0.114;//from webots value
const double neckOffsetX = 0.023;//from webots value

const double shoulderOffsetX = 0;//From webots value
const double shoulderOffsetY = .233;  //from actual robot
const double shoulderOffsetZ = .144;

const double upperArmLength = .246;
const double elbowOffsetX = .030; //Elbow offset
const double lowerArmLength = .190; //From actual 7DOF robot

const double hipOffsetX = -0.001;//From webots model
const double hipOffsetY = 0.094;
const double hipOffsetZ = 0.384;//calculated from webots model

const double thighLength = 0.379;
const double tibiaLength = 0.380;
const double footHeight = 0.04869;//calculated from webots model
const double kneeOffsetX = 0.0;
*/

//================================================================
//THOR-OP values

const double neckOffsetZ = .170+.161; //Webots value
const double neckOffsetX = 0;

const double shoulderOffsetX = 0;      //Webots value
const double shoulderOffsetY = 0.216; //Webots value
const double shoulderOffsetZ = 0.162; //Webots value

const double upperArmLength = .246;
const double elbowOffsetX =   .030; 
const double lowerArmLength = .190; //Measured from robot

const double handOffsetX = 0.245; //Measured from robot
const double handOffsetY = 0.035; //Measured from robot
const double handOffsetZ = 0;

const double hipOffsetX = 0;
const double hipOffsetY = 0.072;
const double hipOffsetZ = 0.270; //Webots value

const double thighLength = 0.30;
const double tibiaLength = 0.30;
const double kneeOffsetX = 0.03;
const double footHeight = 0.118;

//=================================================================

const double dThigh = sqrt(thighLength*thighLength+kneeOffsetX*kneeOffsetX);
const double aThigh = atan(kneeOffsetX/thighLength);
const double dTibia = sqrt(tibiaLength*tibiaLength+kneeOffsetX*kneeOffsetX);
const double aTibia = atan(kneeOffsetX/tibiaLength);

const double dUpperArm = sqrt(upperArmLength*upperArmLength+elbowOffsetX*elbowOffsetX);
const double dLowerArm = sqrt(lowerArmLength*lowerArmLength+elbowOffsetX*elbowOffsetX);
const double aUpperArm = atan(elbowOffsetX/upperArmLength);
const double aLowerArm = atan(elbowOffsetX/lowerArmLength);


//=================================================================
//Those values are used to calculate the multi-body COM of the robot

const double mUpperArm = 2.89;
const double mElbow = 0.13;
const double mLowerArm = 0.81;
const double mWrist = 0.97;
const double mPelvis = 8.0;
const double mTOrso = 9.21;

const double comUpperArmX = 0.1027;
const double comUpperArmZ = -0.008;

const double comElbowX = 0.0159;
const double comElbowZ = 0.0030;

const double comLowerArmX = 0.0464;

const double comWristX = 0.146;
const double comWristZ = -0.0039;

const double comTorsoX = -0.0208;
const double comTorsoZ = 0.1557;

const double comPelvisX = -0.0264;
const double comPelvisZ = -0.1208;














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

Transform THOROP_kinematics_forward_l_wrist(const double *q);
Transform THOROP_kinematics_forward_r_wrist(const double *q);

std::vector<double> THOROP_kinematics_inverse_r_wrist(const Transform trWrist, const double *qOrg, double shoulderYaw);
std::vector<double> THOROP_kinematics_inverse_l_wrist(const Transform trWrist, const double *qOrg, double shoulderYaw);

std::vector<double> THOROP_kinematics_inverse_arm_given_wrist(Transform trArm, const double *qOrg); 

std::vector<double> THOROP_kinematics_inverse_r_arm_7(const Transform trArm, const double *qOrg, double shoulderYaw);
std::vector<double> THOROP_kinematics_inverse_l_arm_7(const Transform trArm, const double *qOrg, double shoulderYaw);

std::vector<double> THOROP_kinematics_inverse_r_leg(const Transform trLeg);
std::vector<double> THOROP_kinematics_inverse_l_leg(const Transform trLeg);
std::vector<double> THOROP_kinematics_inverse_joints(const double *q);

std::vector<double> THOROP_kinematics_com_arm(const double *q, int is_left);  
#endif
