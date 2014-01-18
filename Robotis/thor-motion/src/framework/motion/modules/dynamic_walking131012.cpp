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
#include "framework/motion/modules/dynamic_walking131012.h"

using namespace Thor;

#define PI (3.14159265)

DynamicWalking131012* DynamicWalking131012::m_UniqueInstance = new DynamicWalking131012();

int data_size0;

static PRO54 *g_pro54 = new PRO54();

DynamicWalking131012::DynamicWalking131012()
{
	uID = "DynamicWalking";
	i = 0;

	BALANCE_ENABLE = true;
	BALANCE_KNEE_GAIN = 0.300000;
	BALANCE_ANKLE_PITCH_GAIN = 0.900000;
	BALANCE_HIP_ROLL_GAIN = 0.500000;
	BALANCE_ANKLE_ROLL_GAIN = 1.000000;

	HIP_PITCH_OFFSET = 0.0;

	P_GAIN = 64;
	I_GAIN = 0;
	D_GAIN = 0;

	m_File_Name ="Angles.txt";

}


void DynamicWalking131012::Initialize()
{
	fprintf(stderr, " Start read the angle data\n");
	double AngleData0[35000];
	double a[16];
	int ret = 0;
	FILE *fptr = fopen(m_File_Name, "r");

	int row = 0;
	int column = 0;
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
	for(int i = 0; i < (row-1)*16; i++)
		AngleData[i] = AngleData0[i];

	data_size0 = row -1;

	for(int idx = 0; idx < 16; idx ++)
	{
		m_LastAngle[idx] = AngleData[idx+(data_size0 -1)*16]*251000/PI;
	}


	if(MotionStatus::m_CurrentJoints.size() != 0)
		m_RobotInfo = MotionStatus::m_CurrentJoints;
	else
	{
		fprintf(stderr,"MotionStatus is not initialized");
		return;
	}

	i = 0;

}


void DynamicWalking131012::ReInitialize()
{
	Initialize();
}

void DynamicWalking131012::SetFileName(const char* file_name)
{
	m_File_Name = file_name;
}


void DynamicWalking131012::Process()
{

	//fprintf(stderr, " Start\n");
	//fprintf(stderr, " Process %d\n", i);

	double outValue[16] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	//                     R_HIP_YAW, R_HIP_ROLL, R_HIP_PITCH,    R_KNEE, R_ANKLE_PITCH, R_ANKLE_ROLL,   L_HIP_YAW, L_HIP_ROLL, L_HIP_PITCH,  L_KNEE, L_ANKLE_PITCH, L_ANKLE_ROLL, R_ARM_SWING,  L_ARM_SWING
	int dir[16]          = {      -1,         -1,          -1,        -1,             1,            1,          -1,         -1,           1,       1,            -1,            1,         -1,     1,     -1,      1    };


	if( i >= (data_size0 -1))
	{
		for(int idx = 0; idx < 16; idx ++)
		{
			outValue[idx] = m_LastAngle[idx];
		}
		fprintf(stderr, " Process 1\n" );
	}
	else
	{
		for(int idx = 0; idx < 16; idx ++)
		{
			outValue[idx] = AngleData[idx+i*16]*251000/PI;
		}
	}


    outValue[2] -= (double)dir[2] * HIP_PITCH_OFFSET * (g_pro54->MAX_VALUE)/180.0;
    outValue[8] -= (double)dir[8] * HIP_PITCH_OFFSET * (g_pro54->MAX_VALUE)/180.0;

	if(BALANCE_ENABLE == true)
	{
		double rlGyroErr = MotionStatus::RL_GYRO;
		double fbGyroErr = MotionStatus::FB_GYRO;

		outValue[1] += (int)(dir[1] * rlGyroErr * BALANCE_HIP_ROLL_GAIN*490.23); // R_HIP_ROLL
		outValue[7] += (int)(dir[7] * rlGyroErr * BALANCE_HIP_ROLL_GAIN*490.23); // L_HIP_ROLL

		outValue[5] -= (int)(dir[5] * rlGyroErr * BALANCE_ANKLE_ROLL_GAIN*490.23); // R_ANKLE_ROLL
		outValue[11] -= (int)(dir[11] * rlGyroErr * BALANCE_ANKLE_ROLL_GAIN*490.23); // L_ANKLE_ROLL

		outValue[3] -= (int)(dir[3] * fbGyroErr * BALANCE_KNEE_GAIN*490.23); // R_KNEE
		outValue[9] -= (int)(dir[9] * fbGyroErr * BALANCE_KNEE_GAIN*490.23); // L_KNEE

		outValue[4] -= (int)(dir[4] * fbGyroErr * BALANCE_ANKLE_PITCH_GAIN*490.23); // R_ANKLE_PITCH
		outValue[10] -= (int)(dir[10] * fbGyroErr * BALANCE_ANKLE_PITCH_GAIN*490.23); // L_ANKLE_PITCH

	}


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
			m_RobotInfo[jointIndex].m_Pgain = 16;
			m_RobotInfo[jointIndex].m_Igain = I_GAIN;
			m_RobotInfo[jointIndex].m_Dgain = D_GAIN;
		}
		else if(id == 2)
		{
			m_RobotInfo[jointIndex].m_Value = outValue[13];
			m_RobotInfo[jointIndex].m_Pgain = 16;
			m_RobotInfo[jointIndex].m_Igain = I_GAIN;
			m_RobotInfo[jointIndex].m_Dgain = D_GAIN;
		}
		else if(id == 7)
		{
			m_RobotInfo[jointIndex].m_Value = outValue[14];
			m_RobotInfo[jointIndex].m_Pgain = 16;
			m_RobotInfo[jointIndex].m_Igain = I_GAIN;
			m_RobotInfo[jointIndex].m_Dgain = D_GAIN;
		}
		else if(id == 8)
		{
			m_RobotInfo[jointIndex].m_Value = outValue[15];
			m_RobotInfo[jointIndex].m_Pgain = 16;
			m_RobotInfo[jointIndex].m_Igain = I_GAIN;
			m_RobotInfo[jointIndex].m_Dgain = D_GAIN;
		}

		else
			continue;
	}
	i = i +1;



	//fprintf(stderr, " Start2\n");

}


DynamicWalking131012::~DynamicWalking131012()
{


}

bool DynamicWalking131012::computeIK(double *out, double x, double y, double z, double a, double b, double c)
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
