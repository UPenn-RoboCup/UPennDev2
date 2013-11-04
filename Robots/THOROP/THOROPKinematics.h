#ifndef THOROP7_KINEMATICS_H_
#define THOROP7_KINEMATICS_H_

#include "Transform.h"
#include <stdio.h>
#include <math.h>
#include <vector>

const double PI = 2*asin(1);
const double SQRT2 = sqrt(2);

//THOR-OP values, based on robotis document, and double-checked with actual robot

const double neckOffsetZ = .165+.161; //Webots value
const double neckOffsetX = 0;

const double shoulderOffsetX = 0;    
const double shoulderOffsetY = 0.234;
const double shoulderOffsetZ = 0.165;

const double upperArmLength = .246;
const double elbowOffsetX =   .030; 
const double lowerArmLength = .186; 

const double handOffsetX = 0.245; //Measured from robot
const double handOffsetY = 0.035; //Measured from robot
const double handOffsetZ = 0;

const double hipOffsetX = 0;
const double hipOffsetY = 0.072;
const double hipOffsetZ = 0.282; 

const double thighLength = 0.30;
const double tibiaLength = 0.30;
const double kneeOffsetX = 0.03;
const double footHeight = 0.118; // Webots value

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
const double mTorso = 9.21;

const double mUpperLeg = 4.28;
const double mLowerLeg = 2.24;
const double mFoot = 1.74;

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


const double comUpperLegX = -0.0082;
const double comUpperLegY = 0.0211;
const double comUpperLegZ = -0.124;;

const double comLowerLegX = 0.0074;
const double comLowerLegY = -0.0313;
const double comLowerLegZ = -0.1796;

const double comFootX = -0.0048;
const double comFootZ = -0.0429;







const double servoOffset[] = {
  0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
  0.0, 0.0, 0.0, 0.0, 0.0, 0.0
};

void printTransform(Transform tr);
void printVector(std::vector<double> v);

Transform THOROP_kinematics_forward_head(const double *q);
//Transform THOROP_kinematics_forward_l_arm(const double *q);
//Transform THOROP_kinematics_forward_r_arm(const double *q);
Transform THOROP_kinematics_forward_l_leg(const double *q);
Transform THOROP_kinematics_forward_r_leg(const double *q);
std::vector<double> THOROP_kinematics_forward_joints(const double *r);


Transform THOROP_kinematics_forward_l_arm_7(const double *q, double bodyPitch, const double *qWaist);
Transform THOROP_kinematics_forward_r_arm_7(const double *q, double bodyPitch, const double *qWaist);

std::vector<double> THOROP_kinematics_inverse_r_arm_7(const Transform trArm, const double *qOrg, double shoulderYaw, double bodyPitch, const double *qWaist);
std::vector<double> THOROP_kinematics_inverse_l_arm_7(const Transform trArm, const double *qOrg, double shoulderYaw, double bodyPitch, const double *qWaist);

Transform THOROP_kinematics_forward_l_wrist(const double *q, double bodyPitch, const double *qWaist);
Transform THOROP_kinematics_forward_r_wrist(const double *q, double bodyPitch, const double *qWaist);

std::vector<double> THOROP_kinematics_inverse_r_wrist(const Transform trWrist, const double *qOrg, double shoulderYaw, double bodyPitch, const double *qWaist);
std::vector<double> THOROP_kinematics_inverse_l_wrist(const Transform trWrist, const double *qOrg, double shoulderYaw, double bodyPitch, const double *qWaist); 

std::vector<double> THOROP_kinematics_inverse_arm_given_wrist(Transform trArm, const double *qOrg, double bodyPitch, const double *qWaist); 



std::vector<double> THOROP_kinematics_inverse_r_leg(const Transform trLeg);
std::vector<double> THOROP_kinematics_inverse_l_leg(const Transform trLeg);
std::vector<double> THOROP_kinematics_inverse_joints(const double *q);

std::vector<double> THOROP_kinematics_com_upperbody(const double *qWaist,const double *qLArm,const double *qRArm, double bodyPitch, double mLHand, double mRHand); 
std::vector<double> THOROP_kinematics_com_leg(const double *q, double bodyPitch, int is_left);  
std::vector<double> THOROP_kinematics_calculate_support_torque(
	const double *qWaist,  const double *qLArm,  const double *qRArm,
  	const double *qLLeg,  const double *qRLeg,  
  	double bodyPitch,   int supportLeg,
  	const double *uTorsoAcc, double mLHand, double mRHand); 


#endif
