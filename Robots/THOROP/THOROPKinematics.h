#ifndef THOROP7_KINEMATICS_H_
#define THOROP7_KINEMATICS_H_

#include "Transform.h"
#include <stdio.h>
#include <math.h>
#include <vector>

enum {LEG_LEFT = 0, LEG_RIGHT = 1};
enum {ARM_LEFT = 0, ARM_RIGHT = 1};

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


//const double lowerArmLength = .186; //Default 7DOF arm
const double lowerArmLength = .250; //LONGARM model

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

//SJ: Measured from NEW (smaller) feet
const double footToeX = 0.130; //from ankle to toe
const double footHeelX = 0.110; //from ankle to heel


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
const double comUpperLegZ = -0.124;

const double comLowerLegX = 0.0074;
const double comLowerLegY = -0.0313;
const double comLowerLegZ = -0.1796;

const double comFootX = -0.0048;
const double comFootZ = -0.0429;



//////////////////////////////////////////////////////////////////////////
// New values for calculate the multi-body COM and ZMP of the robot
// Based on latest robotis information for THOR-OP


//Coordinate:   Y Z

/*
//robotis new masses
const double Mass[22] = {
	0.165, 1.122, 3.432, 2.464, 0.946, 1.133, // Mass of Each Right Leg Part
	0.165, 1.122, 3.432, 2.464, 0.946, 1.133, // Mass of Each Left Leg Part
	3.179, 0.13*1.1, 0.81*1.1, 1.067, // Mass of Each Right Arm Part
	3.179, 0.13*1.1, 0.81*1.1, 1.067, // Mass of Each Left Arm Part
	8.8, 10.131}; // Mass of Each Body Part
*/


const double Mass[22]={
	mUpperLeg,mLowerLeg,mFoot,0,0,0,
	mUpperLeg,mLowerLeg,mFoot,0,0,0,
	mUpperArm,mElbow,mLowerArm,mWrist,
	mUpperArm,mElbow,mLowerArm,mWrist,
	mTorso,
	mPelvis
};


const double g = 9.81;

const double MassBody[2]={
	9.21, //torso
	8.0,  //pelvis	
};
const double bodyCom[2][3]={
	{-0.0208,0,0.1557},	//after shoulder pitch
	{-0.0264,0,-0.1208},//after shoulder roll	
};

//Based on webots mass 
const double MassArm[7]={
	0.1, 2.89, 0.13, 0.81, 0.97, 0.1,	 0.1,	//gripper mass... TBD
};

const double armLink[7][3]={
	{0,0.234,0.165}, //waist-shoulder roll 
	{0,0,0}, //shoulder pitch-shoulder roll
	{0,0,0}, //shouder roll-shoulder yaw
	{0.246,0,0.030},//shoulder yaw-elbow 
	{0.250,0,-0.030},//elbow to wrist yaw 1
	{0,0,0},//wrist yaw1 to wrist roll
	{0,0,0}//wrist roll to wrist yaw2
};
const double rarmLink0[3] = {0,-0.234,0.165};

//Com position from joint center
const double armCom[7][3]={
	{0,0,0},	//after shoulder pitch
	{0.1027,0,-0.008},//after shoulder roll
	{0.246,0,0}, //after shoulder yaw
	{0.0464,0,0},//after elbow
	{-0.040,0,0}, //after wrist yaw 1
	{0,0,0}, //after wrist roll
	{0.095,0,0} //after wrist yaw 2
};


const double MassLeg[6]={
	0.165, 1.122, 3.432, 2.464, 0.946, 1.133
};

const double legLink[7][3]={
	{0,-0.072,-0.282}, //waist-hipyaw
	{0,0,0}, //hip yaw-roll
	{0,0,0}, //hip roll-pitch
	{0.030,0,-0.300}, //hip pitch-knee
	{-0.030,0,-0.300}, //knee-ankle pitch
	{0,0,0}, //ankle pitch-ankle roll
	{0,0,-0.118}, //ankle roll - foot bottom
};

const double rlegLink0[3] = {0,0.072,-0.282};
const double llegLink0[3] = {0,-0.072,-0.282};

const double legCom[12][3]={
	//left
	{0,0,0},	//after hip yaw
	{0,0,0},	//after hip roll
	{-0.029, 0.014,-0.130},	//after hip pitch (upper leg)
	{0.031,  0.019,-0.119},	//after knee (lower leg)
	{0,0,0}, //after ankle pitch
	{0,0,-0.031}, //after ankle pitch	

	//right
	{0,0,0},	//after hip yaw
	{0,0,0},	//after hip roll
	{-0.029, -0.014,-0.130},	//after hip pitch (upper leg)
	{0.031,  -0.019,-0.119},	//after knee (lower leg)
	{0,0,0}, //after ankle pitch
	{0,0,-0.031}, //after ankle pitch	
};





const double comOffsetMm[22][3]={//in mm
	//RLEG
	{0,-18.8,47.8}, 
	{0,21.6,0},
	{-129.22547,-13.9106,-29.4},
	{-119.0632,-19.3734,31.3},
	{0,0,-11.2},
	{-31.49254,0.-5.3},
	//LLEG
	{0,-18.8,47.8}, //in mm?
	{0,-21.6,0},
	{-129.22547,13.9106,-29.4},
	{-119.0632,19.3734,31.3},
	{0,0,-11.2},
	{-31.49254,0.-5.3},
	//RARM
	{-22,143.3,0.0}, 
	{0,0,35.1},
	{0,46.4,0},
	{3.9,146,0},
	//LARM
	{-22,-143.3,0.0}, 
	{0,0,35.1},
	{0,-46.4,0},
	{3.9,146,0},
	//Body upper
	{-26.4,0,161.2},
	//Body lower
	{-20.8,0,155.7}
};

const double servoOffset[] = {
  0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
  0.0, 0.0, 0.0, 0.0, 0.0, 0.0
};





///////////////////////////////////////////////////////////////////////////////////////
// COM and ZMP generation functions
///////////////////////////////////////////////////////////////////////////////////////

Transform THOROP_kinematics_forward_head(const double *q);
Transform THOROP_kinematics_forward_l_leg(const double *q);
Transform THOROP_kinematics_forward_r_leg(const double *q);
std::vector<double> THOROP_kinematics_inverse_r_leg(const Transform trLeg, double aShiftX, double aShiftY);
std::vector<double> THOROP_kinematics_inverse_l_leg(const Transform trLeg, double aShiftX, double aShiftY);

///////////////////////////////////////////////////////////////////////////////////////
// Arm FK / IK
///////////////////////////////////////////////////////////////////////////////////////



Transform THOROP_kinematics_forward_l_arm_7(const double *q, double bodyPitch, const double *qWaist,
	double handOffsetXNew, double handOffsetYNew, double handOffsetZNew);
Transform THOROP_kinematics_forward_r_arm_7(const double *q, double bodyPitch, const double *qWaist,
	double handOffsetXNew, double handOffsetYNew, double handOffsetZNew);

std::vector<double> THOROP_kinematics_inverse_r_arm_7(
	const Transform trArm, const double *qOrg, double shoulderYaw, double bodyPitch, const double *qWaist,
	double handOffsetXNew, double handOffsetYNew, double handOffsetZNew, int flip_shoulderroll);
std::vector<double> THOROP_kinematics_inverse_l_arm_7(
	const Transform trArm, const double *qOrg, double shoulderYaw, double bodyPitch, const double *qWaist,
	double handOffsetXNew, double handOffsetYNew, double handOffsetZNew, int flip_shoulderroll);

std::vector<double> THOROP_kinematics_inverse_arm(Transform trArm, std::vector<double>& qOrg, double shoulderYaw, bool flip_shoulderroll);


///////////////////////////////////////////////////////////////////////////////////////
// Wrist FK / IK
///////////////////////////////////////////////////////////////////////////////////////

//std::vector<double> THOROP_kinematics_inverse_wrist(Transform trWrist, std::vector<double>& qOrg, double shoulderYaw);

std::vector<double> THOROP_kinematics_inverse_wrist(Transform trWrist, int arm, const double *qOrg, double shoulderYaw, double bodyPitch, const double *qWaist); 

Transform THOROP_kinematics_forward_l_wrist(const double *q, double bodyPitch, const double *qWaist);
Transform THOROP_kinematics_forward_r_wrist(const double *q, double bodyPitch, const double *qWaist);

std::vector<double> THOROP_kinematics_inverse_r_wrist(const Transform trWrist, const double *qOrg, double shoulderYaw, double bodyPitch, const double *qWaist);
std::vector<double> THOROP_kinematics_inverse_l_wrist(const Transform trWrist, const double *qOrg, double shoulderYaw, double bodyPitch, const double *qWaist); 
std::vector<double> THOROP_kinematics_inverse_arm_given_wrist(Transform trArm, const double *qOrg, double bodyPitch, const double *qWaist); 



///////////////////////////////////////////////////////////////////////////////////////
// COM and ZMP generation
///////////////////////////////////////////////////////////////////////////////////////

std::vector<double> THOROP_kinematics_calculate_com_positions(
    const double *qWaist,  const double *qLArm,   const double *qRArm,
    const double *qLLeg,   const double *qRLeg,   
    double mLHand, double mRHand, double bodyPitch,
    int use_lleg, int use_rleg
    );





std::vector<double> THOROP_kinematics_calculate_zmp(const double *com0, const double *com1, 
		const double *com2,double dt0, double dt1);

int THOROP_kinematics_check_collision(const double *qLArm,const double *qRArm);
int THOROP_kinematics_check_collision_single(const double *qArm,int is_left);


std::vector<double> THOROP_kinematics_calculate_arm_torque(const double *qArm);
std::vector<double> THOROP_kinematics_calculate_leg_torque(const double *qLeg,int isLeft, const double *com_rest);



#endif
