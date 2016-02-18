#ifndef THOROP7_KINEMATICS_H_
#define THOROP7_KINEMATICS_H_

#include "Transform.h"
#include <stdio.h>
#include <math.h>
#include <vector>


///////////////////////////////////////////////////////
// THIS IS DARWIN-OP header file!!!!!!!!!!!!!!!!!!!!!
// I am too lazy for changing all the names... lol
///////////////////////////////////////////////////////





///////////////////////////////////////////////////
// IK header file for teddy (custom legs and arms)
///////////////////////////////////////////////////


enum {LEG_LEFT = 0, LEG_RIGHT = 1};
enum {ARM_LEFT = 0, ARM_RIGHT = 1};

const double PI = 2*asin(1);
const double SQRT2 = sqrt(2);

//DARWIN-OP model (based on webots data)

const double neckOffsetZ = .026+.0505;
const double neckOffsetX = .013;
const double originOffsetZ = 0.111;

const double shoulderOffsetX = .013;    
const double shoulderOffsetY = 0.082; 
const double shoulderOffsetZ = .026;
const double elbowOffsetX =   .016; 

const double upperArmLengthL = .060;
const double lowerArmLengthL = .129;
const double upperArmLengthR = .060;
const double lowerArmLengthR = .129;

const double handOffsetX = 0;
const double handOffsetY = 0;
const double handOffsetZ = 0; 


const double hipOffsetX = .008;
const double hipOffsetY = 0.037;
const double hipOffsetZ = .096;
const double thighLength = 0.093;
const double tibiaLength = 0.093;
const double kneeOffsetX = 0.00;
const double footHeight = 0.0335;
const double footToeX = 0.0525; 
const double footHeelX = 0.0525; 


//mkw lidar positions
const double chestLidarHingeX = 0.05; 
const double chestLidarX = 0; //after lidar servo
const double chestLidarZ = -0.028; //based on shoulder height
const double headLidarX = 0.10; //based on neck servo

//=================================================================
const double dThigh = sqrt(thighLength*thighLength+kneeOffsetX*kneeOffsetX);
const double aThigh = atan(kneeOffsetX/thighLength);
const double dTibia = sqrt(tibiaLength*tibiaLength+kneeOffsetX*kneeOffsetX);
const double aTibia = atan(kneeOffsetX/tibiaLength);


//=================================================================
//Those values are used to calculate the multi-body COM of the robot

const double g = 9.81;



const double MassBody[2]={0.7,0.275599}; //0.975599
const double MassArmL[7]={0.025913, 0, 0.168377, 0.0592885, 0,0,0};
const double MassArmR[7]={0.025913, 0, 0.168377, 0.0592885, 0,0,0};
const double MassLeg[6]={0.0270692, 0.167108,0.119043,0.0703098,0.167108,0.0794462};



const double bodyCom[2][3]={
	{0,0,0},	 //combined com for torso and waist	
	{0,0,0},	 //combined com for torso and waist	
};

const double InertiaArm[7][6]={
	{0.0000625, 0.0000625, 0.0000625, 0,0,0},
	{0.00180625, 0.00180625, 0.00180625, 0,0,0},
	{0.00008125, 0.00008125, 0.00008125, 0,0,0},
	{0.00050625,0.00050625, 0.00050625, 0,0,0},
	{0.00060625,0.00060625, 0.00060625, 0,0,0},
	{0.0000625,0.0000625,0.0000625, 0,0,0},
	{0.0000625,0.0000625,0.0000625, 0,0,0}
};

const double armLinkL[7][3]={
	{0,shoulderOffsetY,shoulderOffsetZ}, //waist-shoulder roll 
	{0,0,0}, //shoulder pitch-shoulder roll
	{0,0,0}, //shouder roll-shoulder yaw
	{upperArmLengthL,0,elbowOffsetX},//shoulder yaw-elbow 
	{lowerArmLengthL,0,-elbowOffsetX},//elbow to wrist yaw 1
	{0,0,0},//wrist yaw1 to wrist roll
	{0,0,0}//wrist roll to wrist yaw2
};

const double armLinkR[7][3]={
	{0,-shoulderOffsetY,shoulderOffsetZ}, //waist-shoulder roll 
	{0,0,0}, //shoulder pitch-shoulder roll
	{0,0,0}, //shouder roll-shoulder yaw
	{upperArmLengthR,0,elbowOffsetX},//shoulder yaw-elbow 
	{lowerArmLengthR,0,-elbowOffsetX},//elbow to wrist yaw 1
	{0,0,0},//wrist yaw1 to wrist roll
	{0,0,0}//wrist roll to wrist yaw2
};

//Com position from joint center
const double armComL[7][3]={
	{0,0,0},	//after shoulder pitch
	{0,0,0},//after shoulder roll
	{0,0,0}, //after shoulder yaw	
	{0,0,0},//after elbow pitch
	{0,0,0}, //after wrist yaw 1
	{0,0,0}, //after wrist roll
	{0,0,0} //after wrist yaw 2
};


//right arm com should be slightly different, but lets assume they are similar
const double armComR[7][3]={
	{0,0,0},	//after shoulder pitch
	{0,0,0},//after shoulder roll
	{0,0,0}, //after shoulder yaw	
	{0,0,0},//after elbow pitch
	{0,0,0}, //after wrist yaw 1
	{0,0,0}, //after wrist roll
	{0,0,0} //after wrist yaw 2
};



const double legLink[7][3]={
	{0,hipOffsetY,-hipOffsetZ}, //waist-hipyaw
	{0,0,0}, //hip yaw-roll
	{0,0,0}, //hip roll-pitch
	{-kneeOffsetX,0,-thighLength}, //hip pitch-knee
	{kneeOffsetX,0,-tibiaLength}, //knee-ankle pitch
	{0,0,0}, //ankle pitch-ankle roll
	{0,0,-footHeight}, //ankle roll - foot bottom
};

const double llegLink0[3] = {0,hipOffsetY,-hipOffsetZ};
const double rlegLink0[3] = {0,-hipOffsetY,-hipOffsetZ};

const double legCom[12][3]={
	//left
	{0,0,0},	//after hip yaw
	{0,0,0},	//after hip roll
	{0,0,0},	//after hip pitch (upper leg), large battery
	{0,0,0},	//after knee (lower leg)
	{0,0,0}, //after ankle pitch
	{0,0,0}, //after ankle pitch	

	//right
	{0,0,0},	//after hip yaw
	{0,0,0},	//after hip roll
	{0,0,0},	//after hip pitch (upper leg)
	{0,0,0},	//after knee (lower leg)
	{0,0,0}, //after ankle pitch
	{0,0,0}, //after ankle pitch	
};

const double InertiaLeg[12][6]={
	//left
	{0.000103125,0.000103125,0.000103125,0,0,0},
	{0.00070125,0.00070125,0.00070125,0,0,0},
	{0.002145,0.002145,0.002145,0,0,0},
	{0.00154,0.00154,0.00154,0,0,0},
	{0.00059125,0.00059125,0.00059125,0,0,0},
	{0.000708125,0.000708125,0.000708125,0,0,0},

	//right
	{0.000103125,0.000103125,0.000103125,0,0,0},
	{0.00070125,0.00070125,0.00070125,0,0,0},
	{0.002145,0.002145,0.002145,0,0,0},
	{0.00154,0.00154,0.00154,0,0,0},
	{0.00059125,0.00059125,0.00059125,0,0,0},
	{0.000708125,0.000708125,0.000708125,0,0,0}
};


///////////////////////////////////////////////////////////////////////////////////////
// COM and ZMP generation functions
///////////////////////////////////////////////////////////////////////////////////////

Transform THOROP_kinematics_forward_head(const double *q);
Transform THOROP_kinematics_forward_l_leg(const double *q);
Transform THOROP_kinematics_forward_r_leg(const double *q);

std::vector<double> THOROP_kinematics_inverse_leg(const Transform trLeg, int leg, double aShiftX, double aShiftY);
std::vector<double> THOROP_kinematics_inverse_leg_toelift(const Transform trLeg, int leg,double aShiftX, double aShiftY,
		int birdwalk, double anklePitchCurrent,  double toeliftMin);
std::vector<double> THOROP_kinematics_inverse_leg_heellift(const Transform trLeg, int leg, double aShiftX, double aShiftY,
		 int birdwalk, double anklePitchCurrent, double heelliftMin);

std::vector<double> THOROP_kinematics_inverse_r_leg(const Transform trLeg, double aShiftX, double aShiftY);
std::vector<double> THOROP_kinematics_inverse_l_leg(const Transform trLeg, double aShiftX, double aShiftY);

double THOROP_kinematics_inverse_leg_bodyheight_diff(const Transform trLeg, int leg, double aShiftX, double aShiftY);

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

std::vector<double> THOROP_kinematics_inverse_larm_given_wrist(Transform trArm, const double *qOrg, double bodyPitch, const double *qWaist); 
	std::vector<double> THOROP_kinematics_inverse_rarm_given_wrist(Transform trArm, const double *qOrg, double bodyPitch, const double *qWaist); 



///////////////////////////////////////////////////////////////////////////////////////
// COM and ZMP generation
///////////////////////////////////////////////////////////////////////////////////////

std::vector<double> THOROP_kinematics_calculate_com_positions(
    const double *qWaist,  const double *qLArm,   const double *qRArm,
    const double *qLLeg,   const double *qRLeg,   
    double mLHand, double mRHand, double bodyPitch,
    int use_lleg, int use_rleg, int birdwalk
    );


std::vector<double> THOROP_kinematics_calculate_zmp(const double *com0, const double *com1, 
		const double *com2,double dt0, double dt1);

int THOROP_kinematics_check_collision(const double *qLArm,const double *qRArm);
int THOROP_kinematics_check_collision_single(const double *qArm,int is_left);


void THOROP_kinematics_calculate_arm_torque(
	double* stall_torque,double* b_matrx,
	const double *rpyangle,	const double *qArm, int is_left);

void THOROP_kinematics_calculate_arm_torque_adv(
  double* stall_torque,double* acc_torque,double* acc_torque2,const double *rpyangle,
  const double *qArm,const double *qArmVel,const double *qArmAcc,double dq, int is_left);

void THOROP_kinematics_calculate_arm_jacobian(  
  double* ret, const double *qArm, const double *qWaist,const double *rpyangle, 
  double handx, double handy, double handz, int is_left);


void THOROP_kinematics_calculate_leg_torque(
	double* stall_torque,double* b_matrx,
	const double *rpyangle,	const double *qLeg,
	int isLeft, double grf, const double *support);

void THOROP_kinematics_calculate_support_leg_torque(
  double* stall_torque, double* b_matrx,
  const double *rpyangle,const double *qLeg,
  int isLeft, double grf, const double *comUpperBody);


#endif
