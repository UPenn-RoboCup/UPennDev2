/*
 * DynamicWalking.cpp
 *
 *  Created on: 2013. 10. 7.
 *      Author: hjsong
 */

#include <stdio.h>
#include <math.h>
#include "framework/math/vector.h"
#include "framework/math/matrix.h"
#include "framework/motion/PRO54.h"
//#include "MX28.h"
#include "framework/motion/kinematics.h"
#include "framework/motion/modules/dynamic_walking.h"
#include "framework/math/linear_algebra.h"

using namespace Thor;

#define PI (3.1415926535897)
#define pi (3.1415926535897)

DynamicWalking* DynamicWalking::m_UniqueInstance = new DynamicWalking();

int data_size;

static PRO54 *g_pro54 = new PRO54();

//
//double min( double a, double b)
//{
//	if (a < b)
//		return a;
//	else
//		return b;
//}
//
//double max( double a, double b)
//{
//	if (a > b)
//		return a;
//	else
//		return b;
//}
//
//double sign(double a)
//{
//	if(a < 0.0)
//		return -1.0;
//	else
//		return 1.0;
//}
//
//double abs(double a)
//{
//	if(a < 0.0)
//		return -a;
//	else
//		return a;
//}


DynamicWalking::DynamicWalking()
{
	uID = "DynamicWalking";
	data_index = 0;

	DEBUG_PRINT = true;
	BALANCE_ENABLE = true;
	BALANCE_KNEE_GAIN = 0.300000;
	BALANCE_ANKLE_PITCH_GAIN = 0.900000;
	BALANCE_HIP_ROLL_GAIN = 0.500000;
	BALANCE_ANKLE_ROLL_GAIN = 1.000000;

	HIP_PITCH_OFFSET = 0.0;


	WALK_STABILIZER_GAIN_RATIO = 1.0;
	IMU_GYRO_GAIN_RATIO = 7.31;
	FORCE_MOMENT_DISTRIBUTION_RATIO = 0.5;

	BALANCE_X_GAIN =            +20.30*(FORCE_MOMENT_DISTRIBUTION_RATIO)*WALK_STABILIZER_GAIN_RATIO;
	BALANCE_Y_GAIN =            -20.30*(FORCE_MOMENT_DISTRIBUTION_RATIO)*WALK_STABILIZER_GAIN_RATIO;
	BALANCE_PITCH_GAIN =         -0.06*(1-FORCE_MOMENT_DISTRIBUTION_RATIO)*WALK_STABILIZER_GAIN_RATIO;
	BALANCE_ROLL_GAIN =          -0.10*(1-FORCE_MOMENT_DISTRIBUTION_RATIO)*WALK_STABILIZER_GAIN_RATIO;

	FOOT_LANDING_OFFSET_GAIN =   +1.0*0;
	FOOT_LANDING_DETECT_N =   50;

	SYSTEM_CONTROL_UNIT_TIME_SEC = MotionModule::TIME_UNIT / 1000.0;
	FOOT_LANDING_DETECTION_TIME_MAX_SEC = 1.0;

	FOOT_ROLL_ADJUSTMENT_ABS_MAX_RAD = 10.0*PI/180;
	FOOT_PITCH_ADJUSTMENT_ABS_MAX_RAD = 10.0*PI/180;

	COB_X_ADJUSTMENT_ABS_MAX_MM = 50;
	COB_Y_ADJUSTMENT_ABS_MAX_MM = 50;

	foot_landing_detection_time_sec = 0;

	foot_r_roll_landing_offset_rad = 0;
	foot_r_pitch_landing_offset_rad = 0;
	foot_l_roll_landing_offset_rad = 0;
	foot_l_pitch_landing_offset_rad = 0;

	foot_r_roll_adjustment_rad = 0;
	foot_r_pitch_adjustment_rad = 0;
	foot_l_roll_adjustment_rad = 0;
	foot_l_pitch_adjustment_rad = 0;

	cob_x_adjustment_mm = 0;
	cob_y_adjustment_mm = 0;

	gyro_roll_init_rad_per_sec = 0;
	gyro_pitch_init_rad_per_sec = 0;
	gyro_yaw_init_rad_per_sec = 0;

	m_roll_init_rad = 0;
	m_pitch_init_rad = 0;
	m_yaw_init_rad = 0;


	P_GAIN = 64;
	I_GAIN = 0;
	D_GAIN = 0;

	m_AngleData_File_Name = "ForwardWalkingWP.txt";
	m_EndPointeData_File_Name = "ForwardWalkingEP.txt";
	m_BalancingIndexeData_File_Name = "ForwardWalkingBalancingIdxData.txt";


	FZ_WINDOW_SIZE = 3;
	m_right_leg_ft_fz_N_array = 0;
	m_left_leg_ft_fz_N_array = 0;

	idxIncreasement = 0.2;
	m_Real_Running = false;
}


void DynamicWalking::Initialize()
{

	double AngleData0[200000];
	double EndPointData0[200000];
	int BalancingIdxData0[200000];

	double a[16];
	double ep[12];
	double balanceIdx;
	int ret = 0;
	fprintf(stderr, "File Open1\n");
	FILE *fptr = fopen(m_AngleData_File_Name, "r");
	fprintf(stderr, "File Open\n");

	int row = 0;
	int column = 0;
	fprintf(stderr, " Start read the angle data\n");
	while(true)
	{
		ret = fscanf(fptr, "%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t",
				&a[0], &a[1], &a[2], &a[3], &a[4], &a[5], &a[6], &a[7], &a[8], &a[9], &a[10], &a[11], &a[12], &a[13], &a[14], &a[15]);

		if(ret == EOF)
			break;

		for(column = 0 ; column < 16; column++)
			AngleData0[row*16 + column] = a[column];

		row = row + 1;
	}
	fclose(fptr);

	data_size = row -1;

	row = 0;
	fprintf(stderr, " Start read the ep data\n");
	fptr = fopen(m_EndPointeData_File_Name, "r");
	while(true)
	{
		ret = fscanf(fptr, "%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t",
				&ep[0], &ep[1], &ep[2], &ep[3], &ep[4], &ep[5], &ep[6], &ep[7], &ep[8], &ep[9], &ep[10], &ep[11]);

		if(ret == EOF)
			break;

		for(column = 0 ; column < 12; column++)
			EndPointData0[row*12 + column] = ep[column];

		row = row + 1;
	}
	fclose(fptr);

	row = 0;
	fprintf(stderr, " Start read the Balancing Idx data\n");
	fptr = fopen(m_BalancingIndexeData_File_Name, "r");
	while(true)
	{
		ret = fscanf(fptr, "%lf", &balanceIdx);

		if(ret == EOF)
			break;

		BalancingIdxData0[row] = (int)balanceIdx;

		row = row + 1;
	}
	fclose(fptr);


	for(int i = 0; i < (row-1)*16; i++)
		AngleData[i] = AngleData0[i];

	for(int i = 0; i < (row-1)*12; i++)
		EndPointData[i] = EndPointData0[i];

	for(int i = 0; i < row; i++)
		BalancingIdxData[i] = BalancingIdxData0[i];



	for(int idx = 0; idx < 16; idx ++)
	{
		m_LastAngle[idx] = AngleData[idx+(data_size -1)*16]*251000/PI;
	}


	if(MotionStatus::m_CurrentJoints.size() != 0)
		m_RobotInfo = MotionStatus::m_CurrentJoints;
	else
	{
		fprintf(stderr,"MotionStatus is not initialized");
		return;
	}

	data_index = 0;

	for(unsigned int jointIndex = 0; jointIndex < m_RobotInfo.size(); jointIndex++)
	{
		int id = m_RobotInfo[jointIndex].m_ID;

		if(id == 15)
		{
			m_RobotInfo[jointIndex].m_Value = AngleData[0]*251000/PI;
			m_RobotInfo[jointIndex].m_Pgain = P_GAIN;
			m_RobotInfo[jointIndex].m_Igain = I_GAIN;
			m_RobotInfo[jointIndex].m_Dgain = D_GAIN;
		}
		else if( id == 17)
		{
			m_RobotInfo[jointIndex].m_Value = AngleData[1]*251000/PI;
			m_RobotInfo[jointIndex].m_Pgain = P_GAIN;
			m_RobotInfo[jointIndex].m_Igain = I_GAIN;
			m_RobotInfo[jointIndex].m_Dgain = D_GAIN;
		}
		else if( id == 19)
		{
			m_RobotInfo[jointIndex].m_Value = AngleData[2]*251000/PI;
			m_RobotInfo[jointIndex].m_Pgain = P_GAIN;
			m_RobotInfo[jointIndex].m_Igain = I_GAIN;
			m_RobotInfo[jointIndex].m_Dgain = D_GAIN;
		}
		else if(id == 21)
		{
			m_RobotInfo[jointIndex].m_Value = AngleData[3]*251000/PI;
			m_RobotInfo[jointIndex].m_Pgain = P_GAIN;
			m_RobotInfo[jointIndex].m_Igain = I_GAIN;
			m_RobotInfo[jointIndex].m_Dgain = D_GAIN;
		}
		else if(id == 23)
		{
			m_RobotInfo[jointIndex].m_Value = AngleData[4]*251000/PI;
			m_RobotInfo[jointIndex].m_Pgain = P_GAIN;
			m_RobotInfo[jointIndex].m_Igain = I_GAIN;
			m_RobotInfo[jointIndex].m_Dgain = D_GAIN;
		}
		else if(id == 25)
		{
			m_RobotInfo[jointIndex].m_Value = AngleData[5]*251000/PI;
			m_RobotInfo[jointIndex].m_Pgain = P_GAIN;
			m_RobotInfo[jointIndex].m_Igain = I_GAIN;
			m_RobotInfo[jointIndex].m_Dgain = D_GAIN;
		}
		else if(id == 16)
		{
			m_RobotInfo[jointIndex].m_Value = AngleData[6]*251000/PI;
			m_RobotInfo[jointIndex].m_Pgain = P_GAIN;
			m_RobotInfo[jointIndex].m_Igain = I_GAIN;
			m_RobotInfo[jointIndex].m_Dgain = D_GAIN;
		}
		else if( id == 18)
		{
			m_RobotInfo[jointIndex].m_Value = AngleData[7]*251000/PI;
			m_RobotInfo[jointIndex].m_Pgain = P_GAIN;
			m_RobotInfo[jointIndex].m_Igain = I_GAIN;
			m_RobotInfo[jointIndex].m_Dgain = D_GAIN;
		}
		else if( id == 20)
		{
			m_RobotInfo[jointIndex].m_Value = AngleData[8]*251000/PI;
			m_RobotInfo[jointIndex].m_Pgain = P_GAIN;
			m_RobotInfo[jointIndex].m_Igain = I_GAIN;
			m_RobotInfo[jointIndex].m_Dgain = D_GAIN;
		}
		else if(id == 22)
		{
			m_RobotInfo[jointIndex].m_Value = AngleData[9]*251000/PI;
			m_RobotInfo[jointIndex].m_Pgain = P_GAIN;
			m_RobotInfo[jointIndex].m_Igain = I_GAIN;
			m_RobotInfo[jointIndex].m_Dgain = D_GAIN;
		}
		else if(id == 24)
		{
			m_RobotInfo[jointIndex].m_Value = AngleData[10]*251000/PI;
			m_RobotInfo[jointIndex].m_Pgain = P_GAIN;
			m_RobotInfo[jointIndex].m_Igain = I_GAIN;
			m_RobotInfo[jointIndex].m_Dgain = D_GAIN;
		}
		else if(id == 26)
		{
			m_RobotInfo[jointIndex].m_Value = AngleData[11]*251000/PI;
			m_RobotInfo[jointIndex].m_Pgain = P_GAIN;
			m_RobotInfo[jointIndex].m_Igain = I_GAIN;
			m_RobotInfo[jointIndex].m_Dgain = D_GAIN;
		}
		else if(id == 1)
		{
			m_RobotInfo[jointIndex].m_Value = AngleData[12]*251000/PI;
			m_RobotInfo[jointIndex].m_Pgain = 16;
			m_RobotInfo[jointIndex].m_Igain = I_GAIN;
			m_RobotInfo[jointIndex].m_Dgain = D_GAIN;
		}
		else if(id == 2)
		{
			m_RobotInfo[jointIndex].m_Value = AngleData[13]*251000/PI;
			m_RobotInfo[jointIndex].m_Pgain = 16;
			m_RobotInfo[jointIndex].m_Igain = I_GAIN;
			m_RobotInfo[jointIndex].m_Dgain = D_GAIN;
		}
		else if(id == 7)
		{
			m_RobotInfo[jointIndex].m_Value = AngleData[14]*251000/PI;
			m_RobotInfo[jointIndex].m_Pgain = 16;
			m_RobotInfo[jointIndex].m_Igain = I_GAIN;
			m_RobotInfo[jointIndex].m_Dgain = D_GAIN;
		}
		else if(id == 8)
		{
			m_RobotInfo[jointIndex].m_Value = AngleData[15]*251000/PI;
			m_RobotInfo[jointIndex].m_Pgain = 16;
			m_RobotInfo[jointIndex].m_Igain = I_GAIN;
			m_RobotInfo[jointIndex].m_Dgain = D_GAIN;
		}

		else
			continue;
	}

	if(	m_right_leg_ft_fz_N_array != 0)
		delete[] m_right_leg_ft_fz_N_array;
	if(m_left_leg_ft_fz_N_array != 0)
		delete[] m_left_leg_ft_fz_N_array;

	m_right_leg_ft_fz_N_array = new double[FZ_WINDOW_SIZE];
	m_left_leg_ft_fz_N_array = new double[FZ_WINDOW_SIZE];
}


void DynamicWalking::ReInitialize()
{
	Initialize();
}

void DynamicWalking::SetFileName(const char* AngleData_File_Name, const char* EndPointeData_File_Name, const char* BalancingIndexeData_File_Name)
{
	m_AngleData_File_Name = AngleData_File_Name;
	m_EndPointeData_File_Name = EndPointeData_File_Name;
	m_BalancingIndexeData_File_Name = BalancingIndexeData_File_Name;
}

void DynamicWalking::SetInitAngleinRad(double roll_init_rad, double pitch_init_rad)
{
	m_roll_init_rad = roll_init_rad;
	m_pitch_init_rad = pitch_init_rad;
}

int preBalancingIdx = 0;
double data_indexx = 0;
void DynamicWalking::Process()
{

	//fprintf(stderr, " Start\n");
	//fprintf(stderr, " Process %d\n", i);

	static double outValue[16] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	static double ep[12]       = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	static double angle[12]    = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	static int balancingIdx = 0;
	static bool bIslanded = false;

	double ankleLength = Kinematics::ANKLE_LENGTH;

//	double right_leg_fx_N  = MotionStatus::R_LEG_FX;
//	double right_leg_fy_N  = MotionStatus::R_LEG_FY;
//	double right_leg_fz_N  = MotionStatus::R_LEG_FZ;
//	double right_leg_Tx_Nm = MotionStatus::R_LEG_TX;
//	double right_leg_Ty_Nm = MotionStatus::R_LEG_TX;
//
//	double left_leg_fx_N  = MotionStatus::L_LEG_FX;
//	double left_leg_fy_N  = MotionStatus::L_LEG_FY;
//	double left_leg_fz_N  = MotionStatus::L_LEG_FZ;
//	double left_leg_Tx_Nm = MotionStatus::L_LEG_TX;
//	double left_leg_Ty_Nm = MotionStatus::L_LEG_TY;


	double right_leg_fx_N  = 1.0;
	double right_leg_fy_N  = 1.0;
	double right_leg_fz_N  = 1.0;
	double right_leg_Tx_Nm = 1.0;
	double right_leg_Ty_Nm = 1.0;

	double left_leg_fx_N  = 1.0;
	double left_leg_fy_N  = 1.0;
	double left_leg_fz_N  = 1.0;
	double left_leg_Tx_Nm = 1.0;
	double left_leg_Ty_Nm = 1.0;

	if(data_index < FZ_WINDOW_SIZE)
	{
		for(int i = data_index ; i < FZ_WINDOW_SIZE ; i++)
		{
			m_right_leg_ft_fz_N_array[i] = right_leg_fz_N;
			m_left_leg_ft_fz_N_array[i] = left_leg_fz_N;
		}
	}
	else
	{
		for(int i = 1 ; i < FZ_WINDOW_SIZE ; i++)
		{
			m_right_leg_ft_fz_N_array[i-1] = m_right_leg_ft_fz_N_array[i];
			m_left_leg_ft_fz_N_array[i-1] = m_left_leg_ft_fz_N_array[i];
		}
		m_right_leg_ft_fz_N_array[FZ_WINDOW_SIZE - 1] = right_leg_fz_N;
		m_left_leg_ft_fz_N_array[FZ_WINDOW_SIZE - 1] = left_leg_fz_N;
	}


	//                     R_HIP_YAW, R_HIP_ROLL, R_HIP_PITCH,    R_KNEE, R_ANKLE_PITCH, R_ANKLE_ROLL,   L_HIP_YAW, L_HIP_ROLL, L_HIP_PITCH,    L_KNEE, L_ANKLE_PITCH, L_ANKLE_ROLL, R_ARM_SWING,  L_ARM_SWING
	int dir[16]          = {      -1,         -1,          -1,        -1,             1,            1,          -1,         -1,           1,         1,            -1,            1,         -1,     1,     -1,      1    };
	double initAngle[12] = {     0.0,        0.0,      5.7106,   33.5788,       -5.7106,          0.0,         0.0,        0.0,     -5.7106,  -33.5788,        5.7106,          0.0};


//	if( data_index >= (data_size -1))
//	{
//		for(int idx = 0; idx < 16; idx ++)
//		{
//			outValue[idx] = m_LastAngle[idx];
//		}
//		//fprintf(stderr, " Process 1\n" );
//	}
//	else
//	{
//		for(int idx = 0; idx < 16; idx ++)
//		{
//			outValue[idx] = AngleData[idx + data_index*16]*251000/PI;
//		}
//	}
//
//
//    outValue[2] -= (double)dir[2] * HIP_PITCH_OFFSET * (g_pro54->MAX_VALUE)/180.0;
//    outValue[8] -= (double)dir[8] * HIP_PITCH_OFFSET * (g_pro54->MAX_VALUE)/180.0;
//
//	if(BALANCE_ENABLE == true)
//	{
//		double rlGyroErr = MotionStatus::RL_GYRO;
//		double fbGyroErr = MotionStatus::FB_GYRO;
//
//		outValue[1] += (int)(dir[1] * rlGyroErr * BALANCE_HIP_ROLL_GAIN*490.23); // R_HIP_ROLL
//		outValue[7] += (int)(dir[7] * rlGyroErr * BALANCE_HIP_ROLL_GAIN*490.23); // L_HIP_ROLL
//
//		outValue[5] -= (int)(dir[5] * rlGyroErr * BALANCE_ANKLE_ROLL_GAIN*490.23); // R_ANKLE_ROLL
//		outValue[11] -= (int)(dir[11] * rlGyroErr * BALANCE_ANKLE_ROLL_GAIN*490.23); // L_ANKLE_ROLL
//
//		outValue[3] -= (int)(dir[3] * fbGyroErr * BALANCE_KNEE_GAIN*490.23); // R_KNEE
//		outValue[9] -= (int)(dir[9] * fbGyroErr * BALANCE_KNEE_GAIN*490.23); // L_KNEE
//
//		outValue[4] -= (int)(dir[4] * fbGyroErr * BALANCE_ANKLE_PITCH_GAIN*490.23); // R_ANKLE_PITCH
//		outValue[10] -= (int)(dir[10] * fbGyroErr * BALANCE_ANKLE_PITCH_GAIN*490.23); // L_ANKLE_PITCH
//
//	}

    // load the ep, angle and balancingIdx of this step
	if( data_index >= (data_size -1))
	{
		for(int idx = 0; idx < 16; idx ++)
		{
			outValue[idx] = m_LastAngle[idx];
		}
		for(int idx = 0; idx < 12; idx ++)
		{
			ep[idx] = EndPointData[idx + data_index*12];
		}
		//fprintf(stderr, " Process 1\n" );
	}
	else
	{
		for(int idx = 0; idx < 16; idx ++)
		{
			outValue[idx] = AngleData[idx + data_index*16]*g_pro54->MAX_VALUE/PI;
		}
		for(int idx = 0; idx < 12; idx ++)
		{
			ep[idx] = EndPointData[idx + data_index*12];
		}
		balancingIdx = BalancingIdxData[data_index];
	}

	double right_zmpx_sensed_by_ft_mm  =  (1000.0*right_leg_Ty_Nm + ankleLength*right_leg_fx_N)/right_leg_fz_N;
	double right_zmpy_sensed_by_ft_mm  = -(-1000.0*right_leg_Tx_Nm + ankleLength*right_leg_fy_N)/right_leg_fz_N;

	double left_zmpx_sensed_by_ft_mm  =  (1000.0*left_leg_Ty_Nm + ankleLength*left_leg_fx_N)/left_leg_fz_N;
	double left_zmpy_sensed_by_ft_mm  = -(-1000.0*left_leg_Tx_Nm + ankleLength*left_leg_fy_N)/left_leg_fz_N;

	double zmpfz_sensed_by_ft_N  = abs(right_leg_fz_N) + abs(left_leg_fz_N);
	double zmpx_sensed_by_ft_mm  = (right_zmpx_sensed_by_ft_mm + ep[1] - Kinematics::LEG_SIDE_OFFSET/2.0)*right_leg_fz_N + (left_zmpx_sensed_by_ft_mm + ep[7] + Kinematics::LEG_SIDE_OFFSET/2.0) /(right_leg_fz_N + left_leg_fz_N);
	double zmpy_sensed_by_ft_mm  = (right_zmpy_sensed_by_ft_mm + ep[0])*right_leg_fz_N + (left_zmpy_sensed_by_ft_mm + ep[6]) / (right_leg_fz_N + left_leg_fz_N);

	double gyro_roll_rad_per_sec  =  MotionStatus::RL_GYRO * 27.925268 / 512.0;
	double gyro_pitch_rad_per_sec =  MotionStatus::FB_GYRO * 27.925268 / 512.0;

	double gyro_pitch_error_rad_per_sec = gyro_pitch_rad_per_sec;
	double gyro_roll_error_rad_per_sec = gyro_roll_rad_per_sec;

	double roll_rad = MotionStatus::EulerAngleX;
	double pitch_rad = MotionStatus::EulerAngleY;

	double roll_error_rad = m_roll_init_rad - roll_rad;
	double pitch_error_rad = m_pitch_init_rad - pitch_rad;

	if(BALANCE_ENABLE) {
        cob_x_adjustment_mm = (IMU_GYRO_GAIN_RATIO*pitch_error_rad+gyro_pitch_error_rad_per_sec) * BALANCE_X_GAIN;
        cob_y_adjustment_mm = (IMU_GYRO_GAIN_RATIO*roll_error_rad+gyro_roll_error_rad_per_sec) * BALANCE_Y_GAIN;
		switch(balancingIdx){
		case 0:
			if(DEBUG_PRINT)
				fprintf(stderr, "DSP : START\n");
            foot_r_roll_adjustment_rad  = foot_r_roll_landing_offset_rad + (IMU_GYRO_GAIN_RATIO*roll_error_rad+gyro_roll_error_rad_per_sec) * BALANCE_ROLL_GAIN;
            foot_r_pitch_adjustment_rad = foot_r_pitch_landing_offset_rad + (IMU_GYRO_GAIN_RATIO*pitch_error_rad+gyro_pitch_error_rad_per_sec) * BALANCE_PITCH_GAIN;
            foot_l_roll_adjustment_rad  = foot_l_roll_landing_offset_rad + (IMU_GYRO_GAIN_RATIO*roll_error_rad+gyro_roll_error_rad_per_sec) * BALANCE_ROLL_GAIN;
            foot_l_pitch_adjustment_rad = foot_l_pitch_landing_offset_rad + (IMU_GYRO_GAIN_RATIO*pitch_error_rad+gyro_pitch_error_rad_per_sec) * BALANCE_PITCH_GAIN;
			foot_landing_detection_time_sec = 0;
			break;
		case 1:
			if(DEBUG_PRINT)
				fprintf(stderr, "DSP : R--O->L\n");
//            foot_r_roll_adjustment_rad  = 0;
//            foot_r_pitch_adjustment_rad = 0;
//            foot_l_roll_adjustment_rad  = 0;
//            foot_l_pitch_adjustment_rad = 0;
            foot_r_roll_adjustment_rad  = foot_r_roll_landing_offset_rad + (IMU_GYRO_GAIN_RATIO*roll_error_rad+gyro_roll_error_rad_per_sec) * BALANCE_ROLL_GAIN;
            foot_r_pitch_adjustment_rad = foot_r_pitch_landing_offset_rad + (IMU_GYRO_GAIN_RATIO*pitch_error_rad+gyro_pitch_error_rad_per_sec) * BALANCE_PITCH_GAIN;
            foot_l_roll_adjustment_rad  = foot_l_roll_landing_offset_rad + (IMU_GYRO_GAIN_RATIO*roll_error_rad+gyro_roll_error_rad_per_sec) * BALANCE_ROLL_GAIN;
            foot_l_pitch_adjustment_rad = foot_l_pitch_landing_offset_rad + (IMU_GYRO_GAIN_RATIO*pitch_error_rad+gyro_pitch_error_rad_per_sec) * BALANCE_PITCH_GAIN;
			break;
		case 2:
			if(DEBUG_PRINT)
				fprintf(stderr, "SSP : L_BALANCING1\n");
            foot_r_roll_landing_offset_rad = FOOT_LANDING_OFFSET_GAIN*roll_error_rad;
            foot_r_pitch_landing_offset_rad = FOOT_LANDING_OFFSET_GAIN*pitch_error_rad;
            foot_r_roll_adjustment_rad  = foot_r_roll_landing_offset_rad + 1*(IMU_GYRO_GAIN_RATIO*roll_error_rad+gyro_roll_error_rad_per_sec) * BALANCE_ROLL_GAIN;
            foot_r_pitch_adjustment_rad = foot_r_pitch_landing_offset_rad + 1*(IMU_GYRO_GAIN_RATIO*pitch_error_rad+gyro_pitch_error_rad_per_sec) * BALANCE_PITCH_GAIN;
            foot_l_roll_adjustment_rad  = foot_l_roll_landing_offset_rad + (IMU_GYRO_GAIN_RATIO*roll_error_rad+gyro_roll_error_rad_per_sec) * BALANCE_ROLL_GAIN;
            foot_l_pitch_adjustment_rad = foot_l_pitch_landing_offset_rad + (IMU_GYRO_GAIN_RATIO*pitch_error_rad+gyro_pitch_error_rad_per_sec) * BALANCE_PITCH_GAIN;
            foot_landing_detection_time_sec = 0;
			break;
		case 3:
			if(DEBUG_PRINT)
				fprintf(stderr, "SSP : L_BALANCING2\n");
            foot_r_roll_landing_offset_rad = FOOT_LANDING_OFFSET_GAIN*roll_error_rad;
            foot_r_pitch_landing_offset_rad = FOOT_LANDING_OFFSET_GAIN*pitch_error_rad;
            foot_r_roll_adjustment_rad  = foot_r_roll_landing_offset_rad + 1*(IMU_GYRO_GAIN_RATIO*roll_error_rad+gyro_roll_error_rad_per_sec) * BALANCE_ROLL_GAIN;
            foot_r_pitch_adjustment_rad = foot_r_pitch_landing_offset_rad + 1*(IMU_GYRO_GAIN_RATIO*pitch_error_rad+gyro_pitch_error_rad_per_sec) * BALANCE_PITCH_GAIN;
            foot_l_roll_adjustment_rad  = foot_l_roll_landing_offset_rad + (IMU_GYRO_GAIN_RATIO*roll_error_rad+gyro_roll_error_rad_per_sec) * BALANCE_ROLL_GAIN;
            foot_l_pitch_adjustment_rad = foot_l_pitch_landing_offset_rad + (IMU_GYRO_GAIN_RATIO*pitch_error_rad+gyro_pitch_error_rad_per_sec) * BALANCE_PITCH_GAIN;

//            if (BalancingIdxData[data_index+1] == 4 && (right_leg_fz_N) < FOOT_LANDING_DETECT_N)
//            {
//    			if(DEBUG_PRINT)
//    				fprintf(stderr, "Wait DSP : R--O<-L !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
//                if (foot_landing_detection_time_sec < FOOT_LANDING_DETECTION_TIME_MAX_SEC) {
//                	data_index = data_index-1;
//                    foot_landing_detection_time_sec = foot_landing_detection_time_sec + SYSTEM_CONTROL_UNIT_TIME_SEC;
//                }
//                else
//                	if(DEBUG_PRINT)
//                		fprintf(stderr, "Stop waiting DSP : R--O<-L ##################################\n");
//            }
//            if (BalancingIdxData[data_index+1] != 4 && (right_leg_fz_N) > FOOT_LANDING_DETECT_N)
//            {
//            	if(DEBUG_PRINT)
//            		fprintf(stderr, "Jump DSP : R--O<-L ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ \n");
//
//                for(int i = data_index + 1 ; i< data_size ; i++)
//                {
//                    if (BalancingIdxData[i] != 3)
//                    {
//                    	data_index = i;
//                        break;
//                    }
//                }
//            }

            bIslanded = true;
            for(int i = 0; i < FZ_WINDOW_SIZE ; i++ )
            {
            	bIslanded &= (m_right_leg_ft_fz_N_array[i] > FOOT_LANDING_DETECT_N);
            }
            if(DEBUG_PRINT)
            {
            	fprintf(stderr, "rightFZ : %f %f %f\n", m_right_leg_ft_fz_N_array[0], m_right_leg_ft_fz_N_array[1], m_right_leg_ft_fz_N_array[2]);
            }
            if (false && BalancingIdxData[data_index+1] == 4 && bIslanded == false)
            {
            	if(DEBUG_PRINT)
            		fprintf(stderr, "Wait DSP : R--O<-L !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
            	if (foot_landing_detection_time_sec < FOOT_LANDING_DETECTION_TIME_MAX_SEC) {
            		data_index = data_index-1;
            		foot_landing_detection_time_sec = foot_landing_detection_time_sec + SYSTEM_CONTROL_UNIT_TIME_SEC;
            	}
            	else
            		if(DEBUG_PRINT){
            			fprintf(stderr, "Stop waiting DSP : R--O<-L ##################################\n");
            		}
            }
            if(false && BalancingIdxData[data_index+1] != 4 && bIslanded == true)
            {
            	if(DEBUG_PRINT)
            		fprintf(stderr, "Jump DSP : R--O<-L ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ \n");

            	for(int i = data_index + 1 ; i< data_size ; i++)
            	{
            		if (BalancingIdxData[i] != 3)
            		{
            			data_index = i;
            			break;
            		}
            	}
            }

			break;
		case 4:
			if(DEBUG_PRINT)
				fprintf(stderr, "DSP : R--O<-L\n");
//            foot_r_roll_adjustment_rad  = 0;
//            foot_r_pitch_adjustment_rad = 0;
//            foot_l_roll_adjustment_rad  = 0;
//            foot_l_pitch_adjustment_rad = 0;
            foot_r_roll_adjustment_rad  = foot_r_roll_landing_offset_rad + (IMU_GYRO_GAIN_RATIO*roll_error_rad+gyro_roll_error_rad_per_sec) * BALANCE_ROLL_GAIN;
            foot_r_pitch_adjustment_rad = foot_r_pitch_landing_offset_rad + (IMU_GYRO_GAIN_RATIO*pitch_error_rad+gyro_pitch_error_rad_per_sec) * BALANCE_PITCH_GAIN;
            foot_l_roll_adjustment_rad  = foot_l_roll_landing_offset_rad + (IMU_GYRO_GAIN_RATIO*roll_error_rad+gyro_roll_error_rad_per_sec) * BALANCE_ROLL_GAIN;
            foot_l_pitch_adjustment_rad = foot_l_pitch_landing_offset_rad + (IMU_GYRO_GAIN_RATIO*pitch_error_rad+gyro_pitch_error_rad_per_sec) * BALANCE_PITCH_GAIN;
            foot_landing_detection_time_sec = 0;
			break;
		case 5:
			if(DEBUG_PRINT)
				fprintf(stderr, "DSP : R<-O--L\n");
//            foot_r_roll_adjustment_rad  = 0;
//            foot_r_pitch_adjustment_rad = 0;
//            foot_l_roll_adjustment_rad  = 0;
//            foot_l_pitch_adjustment_rad = 0;
            foot_r_roll_adjustment_rad  = foot_r_roll_landing_offset_rad + (IMU_GYRO_GAIN_RATIO*roll_error_rad+gyro_roll_error_rad_per_sec) * BALANCE_ROLL_GAIN;
            foot_r_pitch_adjustment_rad = foot_r_pitch_landing_offset_rad + (IMU_GYRO_GAIN_RATIO*pitch_error_rad+gyro_pitch_error_rad_per_sec) * BALANCE_PITCH_GAIN;
            foot_l_roll_adjustment_rad  = foot_l_roll_landing_offset_rad + (IMU_GYRO_GAIN_RATIO*roll_error_rad+gyro_roll_error_rad_per_sec) * BALANCE_ROLL_GAIN;
            foot_l_pitch_adjustment_rad = foot_l_pitch_landing_offset_rad + (IMU_GYRO_GAIN_RATIO*pitch_error_rad+gyro_pitch_error_rad_per_sec) * BALANCE_PITCH_GAIN;
			break;
		case 6:
			if(DEBUG_PRINT)
				fprintf(stderr, "SSP : R_BALANCING1\n");
            foot_l_roll_landing_offset_rad = FOOT_LANDING_OFFSET_GAIN*roll_error_rad;
            foot_l_pitch_landing_offset_rad = FOOT_LANDING_OFFSET_GAIN*pitch_error_rad;
            foot_r_roll_adjustment_rad  = foot_r_roll_landing_offset_rad + (IMU_GYRO_GAIN_RATIO*roll_error_rad+gyro_roll_error_rad_per_sec) * BALANCE_ROLL_GAIN;
            foot_r_pitch_adjustment_rad = foot_r_pitch_landing_offset_rad + (IMU_GYRO_GAIN_RATIO*pitch_error_rad+gyro_pitch_error_rad_per_sec) * BALANCE_PITCH_GAIN;
            foot_l_roll_adjustment_rad  = foot_l_roll_landing_offset_rad + 1*(IMU_GYRO_GAIN_RATIO*roll_error_rad+gyro_roll_error_rad_per_sec) * BALANCE_ROLL_GAIN;
            foot_l_pitch_adjustment_rad = foot_l_pitch_landing_offset_rad + 1*(IMU_GYRO_GAIN_RATIO*pitch_error_rad+gyro_pitch_error_rad_per_sec) * BALANCE_PITCH_GAIN;
            foot_landing_detection_time_sec = 0;
			break;
		case 7:
			if(DEBUG_PRINT)
				fprintf(stderr, "SSP : R_BALANCING2\n");
            foot_l_roll_landing_offset_rad = FOOT_LANDING_OFFSET_GAIN*roll_error_rad;
            foot_l_pitch_landing_offset_rad = FOOT_LANDING_OFFSET_GAIN*pitch_error_rad;
            foot_r_roll_adjustment_rad  = foot_r_roll_landing_offset_rad + (IMU_GYRO_GAIN_RATIO*roll_error_rad+gyro_roll_error_rad_per_sec) * BALANCE_ROLL_GAIN;
            foot_r_pitch_adjustment_rad = foot_r_pitch_landing_offset_rad + (IMU_GYRO_GAIN_RATIO*pitch_error_rad+gyro_pitch_error_rad_per_sec) * BALANCE_PITCH_GAIN;
            foot_l_roll_adjustment_rad  = foot_l_roll_landing_offset_rad + 1*(IMU_GYRO_GAIN_RATIO*roll_error_rad+gyro_roll_error_rad_per_sec) * BALANCE_ROLL_GAIN;
            foot_l_pitch_adjustment_rad = foot_l_pitch_landing_offset_rad + 1*(IMU_GYRO_GAIN_RATIO*pitch_error_rad+gyro_pitch_error_rad_per_sec) * BALANCE_PITCH_GAIN;

//            if (BalancingIdxData[data_index+1] == 8 && (left_leg_fz_N) < FOOT_LANDING_DETECT_N) {
//    			if(DEBUG_PRINT)
//    				fprintf(stderr, "Wait DSP : R->O--L !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
//                if (foot_landing_detection_time_sec < FOOT_LANDING_DETECTION_TIME_MAX_SEC) {
//                	data_index = data_index-1;
//                    foot_landing_detection_time_sec = foot_landing_detection_time_sec + SYSTEM_CONTROL_UNIT_TIME_SEC;
//                }
//                else
//                	fprintf(stderr, "Stop waiting DSP : R->O--L ##################################\n");
//            }
//
//            if (BalancingIdxData[data_index+1] != 8 && (left_leg_fz_N) > FOOT_LANDING_DETECT_N) {
//    			if(DEBUG_PRINT)
//    				fprintf(stderr, "Jump DSP : R->O--L ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~");
//            	for(int i = data_index + 1 ; i< data_size ; i++) {
//            		if (BalancingIdxData[data_index] != 7){
//            			data_index = i;
//            			break;
//            		}
//            	}
//            }

            bIslanded = true;
            for(int i = 0; i < FZ_WINDOW_SIZE ; i++ )
            {
            	bIslanded &= (m_left_leg_ft_fz_N_array[i] > FOOT_LANDING_DETECT_N);
            }
            if(DEBUG_PRINT)
            {
            	fprintf(stderr, "leftFZ : %f %f %f\n", m_left_leg_ft_fz_N_array[0], m_left_leg_ft_fz_N_array[1], m_left_leg_ft_fz_N_array[2]);
            }
            if (false && BalancingIdxData[data_index+1] == 8 && bIslanded == false) {
            	if(DEBUG_PRINT)
            		fprintf(stderr, "Wait DSP : R->O--L !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
            	if (foot_landing_detection_time_sec < FOOT_LANDING_DETECTION_TIME_MAX_SEC) {
            		data_index = data_index-1;
            		foot_landing_detection_time_sec = foot_landing_detection_time_sec + SYSTEM_CONTROL_UNIT_TIME_SEC;
            	}
            	else
            		fprintf(stderr, "Stop waiting DSP : R->O--L ##################################\n");
            }

            if (false && BalancingIdxData[data_index+1] != 8 && bIslanded == true) {
            	if(DEBUG_PRINT)
            		fprintf(stderr, "Jump DSP : R->O--L ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~");
            	for(int i = data_index + 1 ; i< data_size ; i++) {
            		if (BalancingIdxData[i] != 7){
            			data_index = i;
            			break;
            		}
            	}
            }

			break;
		case 8:
			if(DEBUG_PRINT)
				fprintf(stderr, "DSP : R->O--L");

//            foot_r_roll_adjustment_rad  = 0;
//            foot_r_pitch_adjustment_rad = 0;
//            foot_l_roll_adjustment_rad  = 0;
//            foot_l_pitch_adjustment_rad = 0;
            foot_r_roll_adjustment_rad  = foot_r_roll_landing_offset_rad + (IMU_GYRO_GAIN_RATIO*roll_error_rad+gyro_roll_error_rad_per_sec) * BALANCE_ROLL_GAIN;
            foot_r_pitch_adjustment_rad = foot_r_pitch_landing_offset_rad + (IMU_GYRO_GAIN_RATIO*pitch_error_rad+gyro_pitch_error_rad_per_sec) * BALANCE_PITCH_GAIN;
            foot_l_roll_adjustment_rad  = foot_l_roll_landing_offset_rad + (IMU_GYRO_GAIN_RATIO*roll_error_rad+gyro_roll_error_rad_per_sec) * BALANCE_ROLL_GAIN;
            foot_l_pitch_adjustment_rad = foot_l_pitch_landing_offset_rad + (IMU_GYRO_GAIN_RATIO*pitch_error_rad+gyro_pitch_error_rad_per_sec) * BALANCE_PITCH_GAIN;
			break;
		case 9:
			if(DEBUG_PRINT)
				fprintf(stderr, "DSP : END");

//            foot_r_roll_adjustment_rad  = 0;
//            foot_l_roll_adjustment_rad  = 0;
            foot_r_roll_adjustment_rad  = foot_r_roll_landing_offset_rad + (IMU_GYRO_GAIN_RATIO*roll_error_rad+gyro_roll_error_rad_per_sec) * BALANCE_ROLL_GAIN;
            foot_r_pitch_adjustment_rad = foot_r_pitch_landing_offset_rad + (IMU_GYRO_GAIN_RATIO*pitch_error_rad+gyro_pitch_error_rad_per_sec) * BALANCE_PITCH_GAIN;
            foot_l_roll_adjustment_rad  = foot_l_roll_landing_offset_rad + (IMU_GYRO_GAIN_RATIO*roll_error_rad+gyro_roll_error_rad_per_sec) * BALANCE_ROLL_GAIN;
            foot_l_pitch_adjustment_rad = foot_l_pitch_landing_offset_rad + (IMU_GYRO_GAIN_RATIO*pitch_error_rad+gyro_pitch_error_rad_per_sec) * BALANCE_PITCH_GAIN;
			break;
		default:
			break;
		}


        foot_r_roll_adjustment_rad = sign(foot_r_roll_adjustment_rad)*min(abs(foot_r_roll_adjustment_rad), FOOT_ROLL_ADJUSTMENT_ABS_MAX_RAD);
        foot_r_pitch_adjustment_rad = sign(foot_r_pitch_adjustment_rad)*min(abs(foot_r_pitch_adjustment_rad), FOOT_PITCH_ADJUSTMENT_ABS_MAX_RAD);
        foot_l_roll_adjustment_rad = sign(foot_l_roll_adjustment_rad)*min(abs(foot_l_roll_adjustment_rad), FOOT_ROLL_ADJUSTMENT_ABS_MAX_RAD);
        foot_l_pitch_adjustment_rad = sign(foot_l_pitch_adjustment_rad)*min(abs(foot_l_pitch_adjustment_rad), FOOT_PITCH_ADJUSTMENT_ABS_MAX_RAD);

        cob_x_adjustment_mm = sign(cob_x_adjustment_mm)*min(abs(cob_x_adjustment_mm), COB_X_ADJUSTMENT_ABS_MAX_MM);
        cob_y_adjustment_mm = sign(cob_y_adjustment_mm)*min(abs(cob_y_adjustment_mm), COB_Y_ADJUSTMENT_ABS_MAX_MM);

        if(DEBUG_PRINT)
        	fprintf(stderr, " : %f %f %f %f %f %f %f %f\n", cob_x_adjustment_mm, cob_y_adjustment_mm, foot_r_roll_adjustment_rad*180.0/pi, foot_r_pitch_adjustment_rad*180.0/pi, foot_l_roll_adjustment_rad*180.0/pi, foot_l_pitch_adjustment_rad*180.0/pi,  right_leg_fz_N,  left_leg_fz_N);

        ep[0] -= cob_x_adjustment_mm;
        ep[1] -= cob_y_adjustment_mm;

        ep[3] += foot_r_roll_adjustment_rad;
        ep[4] += foot_r_pitch_adjustment_rad;

        ep[6] -= cob_x_adjustment_mm;
        ep[7] -= cob_y_adjustment_mm;

        ep[9]+= foot_l_roll_adjustment_rad;
        ep[10] += foot_l_pitch_adjustment_rad;
	}


    if((computeIK(&angle[0], ep[0], ep[1], ep[2], ep[3], ep[4], ep[5]) == 1)
        && (computeIK(&angle[6], ep[6], ep[7], ep[8], ep[9], ep[10], ep[11]) == 1))
    {
        for(int i=0; i<12; i++)
            angle[i] *= 180.0 / PI;
    }
	else
	{
		return; // Do not use angle;
	}


    for(int i=0; i<12; i++)
    {
		double offset = (double)dir[i] * angle[i] * (g_pro54->MAX_VALUE)/180.0;
        outValue[i] = g_pro54->Angle2Value(initAngle[i]) + (int)offset;
    }

    outValue[2] -= (double)dir[2] * HIP_PITCH_OFFSET * (g_pro54->MAX_VALUE)/180.0;
    outValue[8] -= (double)dir[8] * HIP_PITCH_OFFSET * (g_pro54->MAX_VALUE)/180.0;

	//set the outvalue;
	for(unsigned int jointIndex = 0; jointIndex < m_RobotInfo.size(); jointIndex++)
	{
		int id = m_RobotInfo[jointIndex].m_ID;

		if(id == 15)
		{
			m_RobotInfo[jointIndex].m_Value = outValue[0];
			m_RobotInfo[jointIndex].m_Pgain = P_GAIN;
			m_RobotInfo[jointIndex].m_Igain = I_GAIN;
			m_RobotInfo[jointIndex].m_Dgain = D_GAIN;
		}
		else if( id == 17)
		{
			m_RobotInfo[jointIndex].m_Value = outValue[1];
			m_RobotInfo[jointIndex].m_Pgain = P_GAIN;
			m_RobotInfo[jointIndex].m_Igain = I_GAIN;
			m_RobotInfo[jointIndex].m_Dgain = D_GAIN;
		}
		else if( id == 19)
		{
			m_RobotInfo[jointIndex].m_Value = outValue[2];
			m_RobotInfo[jointIndex].m_Pgain = P_GAIN;
			m_RobotInfo[jointIndex].m_Igain = I_GAIN;
			m_RobotInfo[jointIndex].m_Dgain = D_GAIN;
		}
		else if(id == 21)
		{
			m_RobotInfo[jointIndex].m_Value = outValue[3];
			m_RobotInfo[jointIndex].m_Pgain = P_GAIN;
			m_RobotInfo[jointIndex].m_Igain = I_GAIN;
			m_RobotInfo[jointIndex].m_Dgain = D_GAIN;
		}
		else if(id == 23)
		{
			m_RobotInfo[jointIndex].m_Value = outValue[4];
			m_RobotInfo[jointIndex].m_Pgain = P_GAIN;
			m_RobotInfo[jointIndex].m_Igain = I_GAIN;
			m_RobotInfo[jointIndex].m_Dgain = D_GAIN;
		}
		else if(id == 25)
		{
			m_RobotInfo[jointIndex].m_Value = outValue[5];
			m_RobotInfo[jointIndex].m_Pgain = P_GAIN;
			m_RobotInfo[jointIndex].m_Igain = I_GAIN;
			m_RobotInfo[jointIndex].m_Dgain = D_GAIN;
		}
		else if(id == 16)
		{
			m_RobotInfo[jointIndex].m_Value = outValue[6];
			m_RobotInfo[jointIndex].m_Pgain = P_GAIN;
			m_RobotInfo[jointIndex].m_Igain = I_GAIN;
			m_RobotInfo[jointIndex].m_Dgain = D_GAIN;
		}
		else if( id == 18)
		{
			m_RobotInfo[jointIndex].m_Value = outValue[7];
			m_RobotInfo[jointIndex].m_Pgain = P_GAIN;
			m_RobotInfo[jointIndex].m_Igain = I_GAIN;
			m_RobotInfo[jointIndex].m_Dgain = D_GAIN;
		}
		else if( id == 20)
		{
			m_RobotInfo[jointIndex].m_Value = outValue[8];
			m_RobotInfo[jointIndex].m_Pgain = P_GAIN;
			m_RobotInfo[jointIndex].m_Igain = I_GAIN;
			m_RobotInfo[jointIndex].m_Dgain = D_GAIN;
		}
		else if(id == 22)
		{
			m_RobotInfo[jointIndex].m_Value = outValue[9];
			m_RobotInfo[jointIndex].m_Pgain = P_GAIN;
			m_RobotInfo[jointIndex].m_Igain = I_GAIN;
			m_RobotInfo[jointIndex].m_Dgain = D_GAIN;
		}
		else if(id == 24)
		{
			m_RobotInfo[jointIndex].m_Value = outValue[10];
			m_RobotInfo[jointIndex].m_Pgain = P_GAIN;
			m_RobotInfo[jointIndex].m_Igain = I_GAIN;
			m_RobotInfo[jointIndex].m_Dgain = D_GAIN;
		}
		else if(id == 26)
		{
			m_RobotInfo[jointIndex].m_Value = outValue[11];
			m_RobotInfo[jointIndex].m_Pgain = P_GAIN;
			m_RobotInfo[jointIndex].m_Igain = I_GAIN;
			m_RobotInfo[jointIndex].m_Dgain = D_GAIN;
		}
		else if(id == 1)
		{
			m_RobotInfo[jointIndex].m_Value = outValue[12];
			m_RobotInfo[jointIndex].m_Pgain = P_GAIN;
			m_RobotInfo[jointIndex].m_Igain = I_GAIN;
			m_RobotInfo[jointIndex].m_Dgain = D_GAIN;
		}
		else if(id == 2)
		{
			m_RobotInfo[jointIndex].m_Value = outValue[13];
			m_RobotInfo[jointIndex].m_Pgain = P_GAIN;
			m_RobotInfo[jointIndex].m_Igain = I_GAIN;
			m_RobotInfo[jointIndex].m_Dgain = D_GAIN;
		}
		else if(id == 7)
		{
			m_RobotInfo[jointIndex].m_Value = outValue[14];
			m_RobotInfo[jointIndex].m_Pgain = P_GAIN;
			m_RobotInfo[jointIndex].m_Igain = I_GAIN;
			m_RobotInfo[jointIndex].m_Dgain = D_GAIN;
		}
		else if(id == 8)
		{
			m_RobotInfo[jointIndex].m_Value = outValue[15];
			m_RobotInfo[jointIndex].m_Pgain = P_GAIN;
			m_RobotInfo[jointIndex].m_Igain = I_GAIN;
			m_RobotInfo[jointIndex].m_Dgain = D_GAIN;
		}

		else
			continue;
	}


	preBalancingIdx = balancingIdx;


	if(m_Real_Running == true)
	{
		//data_index = data_index +1;

		data_indexx += idxIncreasement;
		data_index = (int)data_indexx;

		//fprintf(stderr, " time index : %d\n", data_index);
		if(data_index >= (data_size -1))
		{
			data_index = data_size -1;
			m_Real_Running = false;
			fprintf(stderr,"walking complete\n");
		}
	}

}
void DynamicWalking::Start()
{
	m_Real_Running = true;
	data_index = 0;
	data_indexx = 0;
}
void DynamicWalking::Stop()
{
	m_Real_Running = false;
}

DynamicWalking::~DynamicWalking()
{


}

bool DynamicWalking::computeIK(double *out, double x, double y, double z, double a, double b, double c)
{
	Matrix3D Tad, Tda, Tcd, Tdc, Tac;
	Vector3D vec;
	double _Rac, _Acos, _Atan, _k, _l, _m, _n, _s, _c, _theta;
	double LEG_LENGTH = Kinematics::LEG_LENGTH;
	double THIGH_LENGTH = Kinematics::THIGH_LENGTH;
	double CALF_LENGTH = Kinematics::CALF_LENGTH;
	double ANKLE_LENGTH = Kinematics::ANKLE_LENGTH;

	Tad.SetTransform(Point3D(x, y, z - LEG_LENGTH), Vector3D(a * 180.0 / PI, b * 180.0 / PI, c * 180.0 / PI));

	vec.X = x + Tad.m[2] * ANKLE_LENGTH;
	vec.Y = y + Tad.m[6] * ANKLE_LENGTH;
	vec.Z = (z - LEG_LENGTH) + Tad.m[10] * ANKLE_LENGTH;

	// Get Knee
	_Rac = vec.Length();
	_Acos = acos((_Rac * _Rac - THIGH_LENGTH * THIGH_LENGTH - CALF_LENGTH * CALF_LENGTH) / (2 * THIGH_LENGTH * CALF_LENGTH));
	if(isnan(_Acos) == 1)
		return false;
	*(out + 3) = _Acos;

	// Get Ankle Roll
	Tda = Tad;
	if(Tda.Inverse() == false)
		return false;
	_k = sqrt(Tda.m[7] * Tda.m[7] + Tda.m[11] * Tda.m[11]);
	_l = sqrt(Tda.m[7] * Tda.m[7] + (Tda.m[11] - ANKLE_LENGTH) * (Tda.m[11] - ANKLE_LENGTH));
	_m = (_k * _k - _l * _l - ANKLE_LENGTH * ANKLE_LENGTH) / (2 * _l * ANKLE_LENGTH);
	if(_m > 1.0)
		_m = 1.0;
	else if(_m < -1.0)
		_m = -1.0;
	_Acos = acos(_m);
	if(isnan(_Acos) == 1)
		return false;
	if(Tda.m[7] < 0.0)
		*(out + 5) = -_Acos;
	else
		*(out + 5) = _Acos;

	// Get Hip Yaw
	Tcd.SetTransform(Point3D(0, 0, -ANKLE_LENGTH), Vector3D(*(out + 5) * 180.0 / PI, 0, 0));
	Tdc = Tcd;

	if(Tdc.Inverse() == false)
		return false;
	Tac = Tad * Tdc;

	_Atan = atan2(-Tac.m[1] , Tac.m[5]);
	if(isinf(_Atan) == 1)
		return false;
	*(out) = _Atan;

	// Get Hip Roll
	_Atan = atan2(Tac.m[9], -Tac.m[1] * sin(*(out)) + Tac.m[5] * cos(*(out)));
	if(isinf(_Atan) == 1)
		return false;

	*(out + 1) = _Atan;

	// Get Hip Pitch and Ankle Pitch
	_Atan = atan2(Tac.m[2] * cos(*(out)) + Tac.m[6] * sin(*(out)), Tac.m[0] * cos(*(out)) + Tac.m[4] * sin(*(out)));
	if(isinf(_Atan) == 1)
		return false;
	_theta = _Atan;
	_k = sin(*(out + 3)) * CALF_LENGTH;
	_l = -THIGH_LENGTH - cos(*(out + 3)) * CALF_LENGTH;
	_m = cos(*(out)) * vec.X + sin(*(out)) * vec.Y;
	_n = cos(*(out + 1)) * vec.Z + sin(*(out)) * sin(*(out + 1)) * vec.X - cos(*(out)) * sin(*(out + 1)) * vec.Y;
	_s = (_k * _n + _l * _m) / (_k * _k + _l * _l);
	_c = (_n - _k * _s) / _l;
	_Atan = atan2(_s, _c);
	if(isinf(_Atan) == 1)
		return false;
	*(out + 2) = _Atan;
	*(out + 4) = _theta - *(out + 3) - *(out + 2);

	return true;
}
