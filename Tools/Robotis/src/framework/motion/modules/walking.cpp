/*
 * walking.cpp
 *
 *  Created on: 2013. 2. 4.
 *      Author: hjsong
 */

#include "motion/modules/walking.h"


#include <stdio.h>
#include <math.h>
#include "math/vector.h"
#include "math/matrix.h"
#include "motion/PRO54.h"
//#include "motion/motionstatus.h"
#include "motion/kinematics.h"
 #include <unistd.h>

using namespace Thor;


#define PI (3.14159265)

Walking* Walking::m_UniqueInstance = new Walking();

static PRO54 *g_pro54 = new PRO54();

Walking::Walking()
{
	uID = "Walking";
	X_OFFSET = 0;
	Y_OFFSET = 75.0;
	Z_OFFSET = 60;
    R_OFFSET = 0;
	P_OFFSET = 0;
    A_OFFSET = 0;

    HIP_PITCH_OFFSET = 5.0;
	PERIOD_TIME = 900.0;
	DSP_RATIO = 0.0;
	STEP_FB_RATIO = 0.0;
	Z_MOVE_AMPLITUDE = 0;

    Y_SWAP_AMPLITUDE = 0;
    Z_SWAP_AMPLITUDE = 0;
    PELVIS_OFFSET = 0;
    ARM_SWING_GAIN = 0.0;
	BALANCE_KNEE_GAIN = 0.3;
	BALANCE_ANKLE_PITCH_GAIN = 0.9;
	BALANCE_HIP_ROLL_GAIN = 0.5;
	BALANCE_ANKLE_ROLL_GAIN = 1.0;

	P_GAIN = 64;
    I_GAIN = 0;
    D_GAIN = 0;

	X_MOVE_AMPLITUDE = 0;
	Y_MOVE_AMPLITUDE = 0;
	A_MOVE_AMPLITUDE = 0;
	A_MOVE_AIM_ON = false;
	BALANCE_ENABLE = true;

}

Walking::~Walking()
{
	delete g_pro54;
}

void Walking::LoadINISettings(minIni* ini)
{
    LoadINISettings(ini, WALKING_SECTION);
}

void Walking::LoadINISettings(minIni* ini, const std::string &section)
{
    double value = INVALID_VALUE;

    if((value = ini->getd(section, "x_offset", INVALID_VALUE)) != INVALID_VALUE)                X_OFFSET = value;
    if((value = ini->getd(section, "y_offset", INVALID_VALUE)) != INVALID_VALUE)                Y_OFFSET = value;
    if((value = ini->getd(section, "z_offset", INVALID_VALUE)) != INVALID_VALUE)                Z_OFFSET = value;
    if((value = ini->getd(section, "roll_offset", INVALID_VALUE)) != INVALID_VALUE)             R_OFFSET = value;
    if((value = ini->getd(section, "pitch_offset", INVALID_VALUE)) != INVALID_VALUE)            P_OFFSET = value;
    if((value = ini->getd(section, "yaw_offset", INVALID_VALUE)) != INVALID_VALUE)              A_OFFSET = value;
    if((value = ini->getd(section, "hip_pitch_offset", INVALID_VALUE)) != INVALID_VALUE)        HIP_PITCH_OFFSET = value;
    if((value = ini->getd(section, "period_time", INVALID_VALUE)) != INVALID_VALUE)             PERIOD_TIME = value;
    if((value = ini->getd(section, "dsp_ratio", INVALID_VALUE)) != INVALID_VALUE)               DSP_RATIO = value;
    if((value = ini->getd(section, "step_forward_back_ratio", INVALID_VALUE)) != INVALID_VALUE) STEP_FB_RATIO = value;
    if((value = ini->getd(section, "foot_height", INVALID_VALUE)) != INVALID_VALUE)             Z_MOVE_AMPLITUDE = value;
    if((value = ini->getd(section, "swing_right_left", INVALID_VALUE)) != INVALID_VALUE)        Y_SWAP_AMPLITUDE = value;
    if((value = ini->getd(section, "swing_top_down", INVALID_VALUE)) != INVALID_VALUE)          Z_SWAP_AMPLITUDE = value;
    if((value = ini->getd(section, "pelvis_offset", INVALID_VALUE)) != INVALID_VALUE)           PELVIS_OFFSET = value;
    if((value = ini->getd(section, "arm_swing_gain", INVALID_VALUE)) != INVALID_VALUE)          ARM_SWING_GAIN = value;
    if((value = ini->getd(section, "balance_knee_gain", INVALID_VALUE)) != INVALID_VALUE)       BALANCE_KNEE_GAIN = value;
    if((value = ini->getd(section, "balance_ankle_pitch_gain", INVALID_VALUE)) != INVALID_VALUE)BALANCE_ANKLE_PITCH_GAIN = value;
    if((value = ini->getd(section, "balance_hip_roll_gain", INVALID_VALUE)) != INVALID_VALUE)   BALANCE_HIP_ROLL_GAIN = value;
    if((value = ini->getd(section, "balance_ankle_roll_gain", INVALID_VALUE)) != INVALID_VALUE) BALANCE_ANKLE_ROLL_GAIN = value;

    int ivalue = INVALID_VALUE;

    if((ivalue = ini->geti(section, "p_gain", INVALID_VALUE)) != INVALID_VALUE)                 P_GAIN = ivalue;
    if((ivalue = ini->geti(section, "i_gain", INVALID_VALUE)) != INVALID_VALUE)                 I_GAIN = ivalue;
    if((ivalue = ini->geti(section, "d_gain", INVALID_VALUE)) != INVALID_VALUE)                 D_GAIN = ivalue;
}

void Walking::SaveINISettings(minIni* ini)
{
    SaveINISettings(ini, WALKING_SECTION);
}
void Walking::SaveINISettings(minIni* ini, const std::string &section)
{
    ini->put(section,   "x_offset",                 X_OFFSET);
    ini->put(section,   "y_offset",                 Y_OFFSET);
    ini->put(section,   "z_offset",                 Z_OFFSET);
    ini->put(section,   "roll_offset",              R_OFFSET);
    ini->put(section,   "pitch_offset",             P_OFFSET);
    ini->put(section,   "yaw_offset",               A_OFFSET);
    ini->put(section,   "hip_pitch_offset",         HIP_PITCH_OFFSET);
    ini->put(section,   "period_time",              PERIOD_TIME);
    ini->put(section,   "dsp_ratio",                DSP_RATIO);
    ini->put(section,   "step_forward_back_ratio",  STEP_FB_RATIO);
    ini->put(section,   "foot_height",              Z_MOVE_AMPLITUDE);
    ini->put(section,   "swing_right_left",         Y_SWAP_AMPLITUDE);
    ini->put(section,   "swing_top_down",           Z_SWAP_AMPLITUDE);
    ini->put(section,   "pelvis_offset",            PELVIS_OFFSET);
    ini->put(section,   "arm_swing_gain",           ARM_SWING_GAIN);
    ini->put(section,   "balance_knee_gain",        BALANCE_KNEE_GAIN);
    ini->put(section,   "balance_ankle_pitch_gain", BALANCE_ANKLE_PITCH_GAIN);
    ini->put(section,   "balance_hip_roll_gain",    BALANCE_HIP_ROLL_GAIN);
    ini->put(section,   "balance_ankle_roll_gain",  BALANCE_ANKLE_ROLL_GAIN);

    ini->put(section,   "p_gain",                   P_GAIN);
    ini->put(section,   "i_gain",                   I_GAIN);
    ini->put(section,   "d_gain",                   D_GAIN);
}

double Walking::wsin(double time, double period, double period_shift, double mag, double mag_shift)
{
	return mag * sin(2 * 3.141592 / period * time - period_shift) + mag_shift;
}

bool Walking::computeIK(double *out, double x, double y, double z, double a, double b, double c)
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
    *(out + 1) = _Atan*2.0;

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

void Walking::update_param_time()
{
	m_PeriodTime = PERIOD_TIME;
    m_DSP_Ratio = DSP_RATIO;
    m_SSP_Ratio = 1 - DSP_RATIO;

    m_X_Swap_PeriodTime = m_PeriodTime / 2;
    m_X_Move_PeriodTime = m_PeriodTime * m_SSP_Ratio;
    m_Y_Swap_PeriodTime = m_PeriodTime;
    m_Y_Move_PeriodTime = m_PeriodTime * m_SSP_Ratio;
    m_Z_Swap_PeriodTime = m_PeriodTime / 2;
    m_Z_Move_PeriodTime = m_PeriodTime * m_SSP_Ratio / 2;
    m_A_Move_PeriodTime = m_PeriodTime * m_SSP_Ratio;

    m_SSP_Time = m_PeriodTime * m_SSP_Ratio;
    m_SSP_Time_Start_L = (1 - m_SSP_Ratio) * m_PeriodTime / 4;
    m_SSP_Time_End_L = (1 + m_SSP_Ratio) * m_PeriodTime / 4;
    m_SSP_Time_Start_R = (3 - m_SSP_Ratio) * m_PeriodTime / 4;
    m_SSP_Time_End_R = (3 + m_SSP_Ratio) * m_PeriodTime / 4;

    m_Phase_Time1 = (m_SSP_Time_End_L + m_SSP_Time_Start_L) / 2;
    m_Phase_Time2 = (m_SSP_Time_Start_R + m_SSP_Time_End_L) / 2;
    m_Phase_Time3 = (m_SSP_Time_End_R + m_SSP_Time_Start_R) / 2;

    m_Pelvis_Offset = PELVIS_OFFSET*(g_pro54->MAX_VALUE)/180.0;
    m_Pelvis_Swing = m_Pelvis_Offset * 0.35;
    m_Arm_Swing_Gain = ARM_SWING_GAIN;
}

void Walking::update_param_move()
{
	// Forward/Back
    m_X_Move_Amplitude = X_MOVE_AMPLITUDE;
    m_X_Swap_Amplitude = X_MOVE_AMPLITUDE * STEP_FB_RATIO;

    // Right/Left
    m_Y_Move_Amplitude = Y_MOVE_AMPLITUDE / 2;
    if(m_Y_Move_Amplitude > 0)
        m_Y_Move_Amplitude_Shift = m_Y_Move_Amplitude;
    else
        m_Y_Move_Amplitude_Shift = -m_Y_Move_Amplitude;
    m_Y_Swap_Amplitude = Y_SWAP_AMPLITUDE + m_Y_Move_Amplitude_Shift * 0.04;

    m_Z_Move_Amplitude = Z_MOVE_AMPLITUDE / 2;
    m_Z_Move_Amplitude_Shift = m_Z_Move_Amplitude / 2;
    m_Z_Swap_Amplitude = Z_SWAP_AMPLITUDE;
    m_Z_Swap_Amplitude_Shift = m_Z_Swap_Amplitude;

    // Direction
    if(A_MOVE_AIM_ON == false)
    {
        m_A_Move_Amplitude = A_MOVE_AMPLITUDE * PI / 180.0 / 2;
        if(m_A_Move_Amplitude > 0)
            m_A_Move_Amplitude_Shift = m_A_Move_Amplitude;
        else
            m_A_Move_Amplitude_Shift = -m_A_Move_Amplitude;
    }
    else
    {
        m_A_Move_Amplitude = -A_MOVE_AMPLITUDE * PI / 180.0 / 2;
        if(m_A_Move_Amplitude > 0)
            m_A_Move_Amplitude_Shift = -m_A_Move_Amplitude;
        else
            m_A_Move_Amplitude_Shift = m_A_Move_Amplitude;
    }
}

void Walking::update_param_balance()
{
	m_X_Offset = X_OFFSET;
    m_Y_Offset = Y_OFFSET;
    m_Z_Offset = Z_OFFSET;
    m_R_Offset = R_OFFSET * PI / 180.0;
    m_P_Offset = P_OFFSET * PI / 180.0;
    m_A_Offset = A_OFFSET * PI / 180.0;
    m_Hip_Pitch_Offset = HIP_PITCH_OFFSET*(g_pro54->MAX_VALUE)/180.0;
}

void Walking::Initialize()
{
	X_MOVE_AMPLITUDE   = 0;
    Y_MOVE_AMPLITUDE   = 0;
    A_MOVE_AMPLITUDE   = 0;

	m_Body_Swing_Y = 0;
    m_Body_Swing_Z = 0;

	m_X_Swap_Phase_Shift = PI;
    m_X_Swap_Amplitude_Shift = 0;
    m_X_Move_Phase_Shift = PI / 2;
    m_X_Move_Amplitude_Shift = 0;
    m_Y_Swap_Phase_Shift = 0;
    m_Y_Swap_Amplitude_Shift = 0;
    m_Y_Move_Phase_Shift = PI / 2;
    m_Z_Swap_Phase_Shift = PI * 3 / 2;
    m_Z_Move_Phase_Shift = PI / 2;
    m_A_Move_Phase_Shift = PI / 2;

	m_Ctrl_Running = false;
    m_Real_Running = false;
    m_Time = 0;
    update_param_time();
    update_param_move();

	if(MotionStatus::m_CurrentJoints.size() != 0)
		m_RobotInfo = MotionStatus::m_CurrentJoints;
	else
	{
		fprintf(stderr,"MotionStatus is not initialized");
		return;
	}

	int id = -1;
	for(unsigned int jointIndex = 0; jointIndex < m_RobotInfo.size(); jointIndex ++)
	{
		id = m_RobotInfo[jointIndex].m_ID;
		if(id >= 1 && id <= 26)
		{
			if(id == 1)
			{
				m_RobotInfo[jointIndex].m_Value = m_RobotInfo[jointIndex].m_DXLInfo->Angle2Value(-3.345);
				m_RobotInfo[jointIndex].m_Pgain = 8;
			}
			else if(id ==2)
			{
				m_RobotInfo[jointIndex].m_Value = m_RobotInfo[jointIndex].m_DXLInfo->Angle2Value(-3.687);
				m_RobotInfo[jointIndex].m_Pgain = 8;
			}
			else if(id == 3)
			{
				m_RobotInfo[jointIndex].m_Value = m_RobotInfo[jointIndex].m_DXLInfo->Angle2Value(-63.873);
				m_RobotInfo[jointIndex].m_Pgain = 8;
			}
			else if(id == 4)
			{
				m_RobotInfo[jointIndex].m_Value = m_RobotInfo[jointIndex].m_DXLInfo->Angle2Value(63.873);
				m_RobotInfo[jointIndex].m_Pgain = 8;

			}
			else if(id == 5)
			{
				m_RobotInfo[jointIndex].m_Value = m_RobotInfo[jointIndex].m_DXLInfo->Angle2Value(90.0);
				m_RobotInfo[jointIndex].m_Pgain = 32;

			}
			else if(id == 6)
			{
				m_RobotInfo[jointIndex].m_Value = m_RobotInfo[jointIndex].m_DXLInfo->Angle2Value(-90.0);
				m_RobotInfo[jointIndex].m_Pgain = 32;

			}
			else if(id == 7)
			{
				m_RobotInfo[jointIndex].m_Value = m_RobotInfo[jointIndex].m_DXLInfo->Angle2Value(39.300);
				m_RobotInfo[jointIndex].m_Pgain = 8;
			}
			else if(id == 8)
			{
				m_RobotInfo[jointIndex].m_Value = m_RobotInfo[jointIndex].m_DXLInfo->Angle2Value(-39.593);
				m_RobotInfo[jointIndex].m_Pgain = 8;
			}
			else if(id == 9)
			{
				m_RobotInfo[jointIndex].m_Value = -75937;
				m_RobotInfo[jointIndex].m_Pgain = 8;
			}
			else if(id == 10)
			{
				m_RobotInfo[jointIndex].m_Value = 75937;
				m_RobotInfo[jointIndex].m_Pgain = 8;
			}
		}
	}

    Process();
}

void Walking::Start()
{
	if(MotionStatus::m_CurrentJoints.size() != 0)
		m_RobotInfo = MotionStatus::m_CurrentJoints;
	else
	{
		fprintf(stderr,"MotionStatus is not initialized");
		return;
	}

	m_Ctrl_Running = true;
    m_Real_Running = true;
}

void Walking::Stop()
{
	m_Ctrl_Running = false;
}

bool Walking::IsRunning()
{
	return m_Real_Running;
}

void Walking::Process()
{
	double x_swap, y_swap, z_swap, a_swap, b_swap, c_swap;
    double x_move_r, y_move_r, z_move_r, a_move_r, b_move_r, c_move_r;
    double x_move_l, y_move_l, z_move_l, a_move_l, b_move_l, c_move_l;
    double pelvis_offset_r, pelvis_offset_l;
    double angle[14], ep[12];
	double offset;
	double TIME_UNIT = MotionModule::TIME_UNIT;

	//벨트모듈 방향 동일
	//                     R_HIP_YAW, R_HIP_ROLL, R_HIP_PITCH,    R_KNEE, R_ANKLE_PITCH, R_ANKLE_ROLL,   L_HIP_YAW, L_HIP_ROLL, L_HIP_PITCH,    L_KNEE, L_ANKLE_PITCH,  L_ANKLE_ROLL, R_ARM_SWING,  L_ARM_SWING
	int dir[14]          = {      -1,         -1,          -1,        -1,             1,            1,          -1,         -1,           1,         1,             -1,            1,           1,           -1    };
    double initAngle[14] = {     0.0,        0.0,      5.7106,   33.5788,       -5.7106,          0.0,         0.0,        0.0,     -5.7106,  -33.5788,         5.7106,          0.0,     -43.345,       46.313    };

	//벨트모듈 방향 반대
//	                        R_HIP_YAW, R_HIP_ROLL, R_HIP_PITCH,    R_KNEE, R_ANKLE_PITCH, R_ANKLE_ROLL,   L_HIP_YAW, L_HIP_ROLL, L_HIP_PITCH,    L_KNEE, L_ANKLE_PITCH,  L_ANKLE_ROLL, R_ARM_SWING,  L_ARM_SWING
//	int dir[14]          = {      -1,         -1,          1,        -1,             -1,            1,          -1,         -1,           -1,         1,             1,            1,           1,           -1    };
//    double initAngle[14] = {     0.0,        0.0,      -5.7106,   33.5788,        5.7106,          0.0,         0.0,        0.0,     5.7106,  -33.5788,         -5.7106,          0.0,     -33.345,       36.313    };

//	//오른다리 반대 왼다리 동일
//	//  	                     R_HIP_YAW, R_HIP_ROLL, R_HIP_PITCH,    R_KNEE, R_ANKLE_PITCH, R_ANKLE_ROLL,   L_HIP_YAW, L_HIP_ROLL, L_HIP_PITCH,    L_KNEE, L_ANKLE_PITCH,  L_ANKLE_ROLL, R_ARM_SWING,  L_ARM_SWING
//	int dir[14]          = {      -1,         -1,          1,        -1,             -1,            1,          -1,         -1,           1,         1,            -1,            1,           1,           -1    };
//	double initAngle[14] = {     0.0,        0.0,      -5.7106,   33.5788,        5.7106,          0.0,         0.0,        0.0,     -5.7106,  -33.5788,         5.7106,          0.0,     -33.345,       36.313  };

	//오른다리 동일 왼다리 반대
	//  	                     R_HIP_YAW, R_HIP_ROLL, R_HIP_PITCH,    R_KNEE, R_ANKLE_PITCH, R_ANKLE_ROLL,   L_HIP_YAW, L_HIP_ROLL, L_HIP_PITCH,    L_KNEE, L_ANKLE_PITCH,  L_ANKLE_ROLL, R_ARM_SWING,  L_ARM_SWING
//	int dir[14]          = {      -1,         -1,          -1,        -1,             1,            1,          -1,         -1,           -1,         1,             1,            1,           1,           -1    };
//    double initAngle[14] = {     0.0,        0.0,       5.7106,   33.5788,        -5.7106,          0.0,         0.0,        0.0,      5.7106,  -33.5788,         -5.7106,          0.0,     -33.345,       36.313    };



	int outValue[14];

    // Update walk parameters
    if(m_Time == 0)
    {
        update_param_time();
        m_Phase = PHASE0;
        if(m_Ctrl_Running == false)
        {
            if(m_X_Move_Amplitude == 0 && m_Y_Move_Amplitude == 0 && m_A_Move_Amplitude == 0)
            {
                m_Real_Running = false;
            }
            else
            {
                X_MOVE_AMPLITUDE = 0;
                Y_MOVE_AMPLITUDE = 0;
                A_MOVE_AMPLITUDE = 0;
            }
        }
    }
    else if(m_Time >= (m_Phase_Time1 - TIME_UNIT/2) && m_Time < (m_Phase_Time1 + TIME_UNIT/2))
    {
        update_param_move();
        m_Phase = PHASE1;
    }
    else if(m_Time >= (m_Phase_Time2 - TIME_UNIT/2) && m_Time < (m_Phase_Time2 + TIME_UNIT/2))
    {
        update_param_time();
        m_Time = m_Phase_Time2;
        m_Phase = PHASE2;
        if(m_Ctrl_Running == false)
        {
            if(m_X_Move_Amplitude == 0 && m_Y_Move_Amplitude == 0 && m_A_Move_Amplitude == 0)
            {
                m_Real_Running = false;
            }
            else
            {
                X_MOVE_AMPLITUDE = 0;
                Y_MOVE_AMPLITUDE = 0;
                A_MOVE_AMPLITUDE = 0;
            }
        }
    }
    else if(m_Time >= (m_Phase_Time3 - TIME_UNIT/2) && m_Time < (m_Phase_Time3 + TIME_UNIT/2))
    {
        update_param_move();
        m_Phase = PHASE3;
    }
    update_param_balance();

    // Compute endpoints
    x_swap = wsin(m_Time, m_X_Swap_PeriodTime, m_X_Swap_Phase_Shift, m_X_Swap_Amplitude, m_X_Swap_Amplitude_Shift);
    y_swap = wsin(m_Time, m_Y_Swap_PeriodTime, m_Y_Swap_Phase_Shift, m_Y_Swap_Amplitude, m_Y_Swap_Amplitude_Shift);
    z_swap = wsin(m_Time, m_Z_Swap_PeriodTime, m_Z_Swap_Phase_Shift, m_Z_Swap_Amplitude, m_Z_Swap_Amplitude_Shift);
    a_swap = 0;
    b_swap = 0;
    c_swap = 0;

    if(m_Time <= m_SSP_Time_Start_L)
    {
        x_move_l = wsin(m_SSP_Time_Start_L, m_X_Move_PeriodTime, m_X_Move_Phase_Shift + 2 * PI / m_X_Move_PeriodTime * m_SSP_Time_Start_L, m_X_Move_Amplitude, m_X_Move_Amplitude_Shift);
        y_move_l = wsin(m_SSP_Time_Start_L, m_Y_Move_PeriodTime, m_Y_Move_Phase_Shift + 2 * PI / m_Y_Move_PeriodTime * m_SSP_Time_Start_L, m_Y_Move_Amplitude, m_Y_Move_Amplitude_Shift);
        z_move_l = wsin(m_SSP_Time_Start_L, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * PI / m_Z_Move_PeriodTime * m_SSP_Time_Start_L, m_Z_Move_Amplitude, m_Z_Move_Amplitude_Shift);
        c_move_l = wsin(m_SSP_Time_Start_L, m_A_Move_PeriodTime, m_A_Move_Phase_Shift + 2 * PI / m_A_Move_PeriodTime * m_SSP_Time_Start_L, m_A_Move_Amplitude, m_A_Move_Amplitude_Shift);
        x_move_r = wsin(m_SSP_Time_Start_L, m_X_Move_PeriodTime, m_X_Move_Phase_Shift + 2 * PI / m_X_Move_PeriodTime * m_SSP_Time_Start_L, -m_X_Move_Amplitude, -m_X_Move_Amplitude_Shift);
        y_move_r = wsin(m_SSP_Time_Start_L, m_Y_Move_PeriodTime, m_Y_Move_Phase_Shift + 2 * PI / m_Y_Move_PeriodTime * m_SSP_Time_Start_L, -m_Y_Move_Amplitude, -m_Y_Move_Amplitude_Shift);
        z_move_r = wsin(m_SSP_Time_Start_R, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * PI / m_Z_Move_PeriodTime * m_SSP_Time_Start_R, m_Z_Move_Amplitude, m_Z_Move_Amplitude_Shift);
        c_move_r = wsin(m_SSP_Time_Start_L, m_A_Move_PeriodTime, m_A_Move_Phase_Shift + 2 * PI / m_A_Move_PeriodTime * m_SSP_Time_Start_L, -m_A_Move_Amplitude, -m_A_Move_Amplitude_Shift);
        pelvis_offset_l = 0;
        pelvis_offset_r = 0;
    }
    else if(m_Time <= m_SSP_Time_End_L)
    {
        x_move_l = wsin(m_Time, m_X_Move_PeriodTime, m_X_Move_Phase_Shift + 2 * PI / m_X_Move_PeriodTime * m_SSP_Time_Start_L, m_X_Move_Amplitude, m_X_Move_Amplitude_Shift);
        y_move_l = wsin(m_Time, m_Y_Move_PeriodTime, m_Y_Move_Phase_Shift + 2 * PI / m_Y_Move_PeriodTime * m_SSP_Time_Start_L, m_Y_Move_Amplitude, m_Y_Move_Amplitude_Shift);
        z_move_l = wsin(m_Time, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * PI / m_Z_Move_PeriodTime * m_SSP_Time_Start_L, m_Z_Move_Amplitude, m_Z_Move_Amplitude_Shift);
        c_move_l = wsin(m_Time, m_A_Move_PeriodTime, m_A_Move_Phase_Shift + 2 * PI / m_A_Move_PeriodTime * m_SSP_Time_Start_L, m_A_Move_Amplitude, m_A_Move_Amplitude_Shift);
        x_move_r = wsin(m_Time, m_X_Move_PeriodTime, m_X_Move_Phase_Shift + 2 * PI / m_X_Move_PeriodTime * m_SSP_Time_Start_L, -m_X_Move_Amplitude, -m_X_Move_Amplitude_Shift);
        y_move_r = wsin(m_Time, m_Y_Move_PeriodTime, m_Y_Move_Phase_Shift + 2 * PI / m_Y_Move_PeriodTime * m_SSP_Time_Start_L, -m_Y_Move_Amplitude, -m_Y_Move_Amplitude_Shift);
        z_move_r = wsin(m_SSP_Time_Start_R, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * PI / m_Z_Move_PeriodTime * m_SSP_Time_Start_R, m_Z_Move_Amplitude, m_Z_Move_Amplitude_Shift);
        c_move_r = wsin(m_Time, m_A_Move_PeriodTime, m_A_Move_Phase_Shift + 2 * PI / m_A_Move_PeriodTime * m_SSP_Time_Start_L, -m_A_Move_Amplitude, -m_A_Move_Amplitude_Shift);
        pelvis_offset_l = wsin(m_Time, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * PI / m_Z_Move_PeriodTime * m_SSP_Time_Start_L, m_Pelvis_Swing / 2, m_Pelvis_Swing / 2);
        pelvis_offset_r = wsin(m_Time, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * PI / m_Z_Move_PeriodTime * m_SSP_Time_Start_L, -m_Pelvis_Offset / 2, -m_Pelvis_Offset / 2);
    }
    else if(m_Time <= m_SSP_Time_Start_R)
    {
        x_move_l = wsin(m_SSP_Time_End_L, m_X_Move_PeriodTime, m_X_Move_Phase_Shift + 2 * PI / m_X_Move_PeriodTime * m_SSP_Time_Start_L, m_X_Move_Amplitude, m_X_Move_Amplitude_Shift);
        y_move_l = wsin(m_SSP_Time_End_L, m_Y_Move_PeriodTime, m_Y_Move_Phase_Shift + 2 * PI / m_Y_Move_PeriodTime * m_SSP_Time_Start_L, m_Y_Move_Amplitude, m_Y_Move_Amplitude_Shift);
        z_move_l = wsin(m_SSP_Time_End_L, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * PI / m_Z_Move_PeriodTime * m_SSP_Time_Start_L, m_Z_Move_Amplitude, m_Z_Move_Amplitude_Shift);
        c_move_l = wsin(m_SSP_Time_End_L, m_A_Move_PeriodTime, m_A_Move_Phase_Shift + 2 * PI / m_A_Move_PeriodTime * m_SSP_Time_Start_L, m_A_Move_Amplitude, m_A_Move_Amplitude_Shift);
        x_move_r = wsin(m_SSP_Time_End_L, m_X_Move_PeriodTime, m_X_Move_Phase_Shift + 2 * PI / m_X_Move_PeriodTime * m_SSP_Time_Start_L, -m_X_Move_Amplitude, -m_X_Move_Amplitude_Shift);
        y_move_r = wsin(m_SSP_Time_End_L, m_Y_Move_PeriodTime, m_Y_Move_Phase_Shift + 2 * PI / m_Y_Move_PeriodTime * m_SSP_Time_Start_L, -m_Y_Move_Amplitude, -m_Y_Move_Amplitude_Shift);
        z_move_r = wsin(m_SSP_Time_Start_R, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * PI / m_Z_Move_PeriodTime * m_SSP_Time_Start_R, m_Z_Move_Amplitude, m_Z_Move_Amplitude_Shift);
        c_move_r = wsin(m_SSP_Time_End_L, m_A_Move_PeriodTime, m_A_Move_Phase_Shift + 2 * PI / m_A_Move_PeriodTime * m_SSP_Time_Start_L, -m_A_Move_Amplitude, -m_A_Move_Amplitude_Shift);
        pelvis_offset_l = 0;
        pelvis_offset_r = 0;
    }
    else if(m_Time <= m_SSP_Time_End_R)
    {
        x_move_l = wsin(m_Time, m_X_Move_PeriodTime, m_X_Move_Phase_Shift + 2 * PI / m_X_Move_PeriodTime * m_SSP_Time_Start_R + PI, m_X_Move_Amplitude, m_X_Move_Amplitude_Shift);
        y_move_l = wsin(m_Time, m_Y_Move_PeriodTime, m_Y_Move_Phase_Shift + 2 * PI / m_Y_Move_PeriodTime * m_SSP_Time_Start_R + PI, m_Y_Move_Amplitude, m_Y_Move_Amplitude_Shift);
        z_move_l = wsin(m_SSP_Time_End_L, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * PI / m_Z_Move_PeriodTime * m_SSP_Time_Start_L, m_Z_Move_Amplitude, m_Z_Move_Amplitude_Shift);
        c_move_l = wsin(m_Time, m_A_Move_PeriodTime, m_A_Move_Phase_Shift + 2 * PI / m_A_Move_PeriodTime * m_SSP_Time_Start_R + PI, m_A_Move_Amplitude, m_A_Move_Amplitude_Shift);
        x_move_r = wsin(m_Time, m_X_Move_PeriodTime, m_X_Move_Phase_Shift + 2 * PI / m_X_Move_PeriodTime * m_SSP_Time_Start_R + PI, -m_X_Move_Amplitude, -m_X_Move_Amplitude_Shift);
        y_move_r = wsin(m_Time, m_Y_Move_PeriodTime, m_Y_Move_Phase_Shift + 2 * PI / m_Y_Move_PeriodTime * m_SSP_Time_Start_R + PI, -m_Y_Move_Amplitude, -m_Y_Move_Amplitude_Shift);
        z_move_r = wsin(m_Time, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * PI / m_Z_Move_PeriodTime * m_SSP_Time_Start_R, m_Z_Move_Amplitude, m_Z_Move_Amplitude_Shift);
        c_move_r = wsin(m_Time, m_A_Move_PeriodTime, m_A_Move_Phase_Shift + 2 * PI / m_A_Move_PeriodTime * m_SSP_Time_Start_R + PI, -m_A_Move_Amplitude, -m_A_Move_Amplitude_Shift);
        pelvis_offset_l = wsin(m_Time, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * PI / m_Z_Move_PeriodTime * m_SSP_Time_Start_R, m_Pelvis_Offset / 2, m_Pelvis_Offset / 2);
        pelvis_offset_r = wsin(m_Time, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * PI / m_Z_Move_PeriodTime * m_SSP_Time_Start_R, -m_Pelvis_Swing / 2, -m_Pelvis_Swing / 2);
    }
    else
    {
        x_move_l = wsin(m_SSP_Time_End_R, m_X_Move_PeriodTime, m_X_Move_Phase_Shift + 2 * PI / m_X_Move_PeriodTime * m_SSP_Time_Start_R + PI, m_X_Move_Amplitude, m_X_Move_Amplitude_Shift);
        y_move_l = wsin(m_SSP_Time_End_R, m_Y_Move_PeriodTime, m_Y_Move_Phase_Shift + 2 * PI / m_Y_Move_PeriodTime * m_SSP_Time_Start_R + PI, m_Y_Move_Amplitude, m_Y_Move_Amplitude_Shift);
        z_move_l = wsin(m_SSP_Time_End_L, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * PI / m_Z_Move_PeriodTime * m_SSP_Time_Start_L, m_Z_Move_Amplitude, m_Z_Move_Amplitude_Shift);
        c_move_l = wsin(m_SSP_Time_End_R, m_A_Move_PeriodTime, m_A_Move_Phase_Shift + 2 * PI / m_A_Move_PeriodTime * m_SSP_Time_Start_R + PI, m_A_Move_Amplitude, m_A_Move_Amplitude_Shift);
        x_move_r = wsin(m_SSP_Time_End_R, m_X_Move_PeriodTime, m_X_Move_Phase_Shift + 2 * PI / m_X_Move_PeriodTime * m_SSP_Time_Start_R + PI, -m_X_Move_Amplitude, -m_X_Move_Amplitude_Shift);
        y_move_r = wsin(m_SSP_Time_End_R, m_Y_Move_PeriodTime, m_Y_Move_Phase_Shift + 2 * PI / m_Y_Move_PeriodTime * m_SSP_Time_Start_R + PI, -m_Y_Move_Amplitude, -m_Y_Move_Amplitude_Shift);
        z_move_r = wsin(m_SSP_Time_End_R, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * PI / m_Z_Move_PeriodTime * m_SSP_Time_Start_R, m_Z_Move_Amplitude, m_Z_Move_Amplitude_Shift);
        c_move_r = wsin(m_SSP_Time_End_R, m_A_Move_PeriodTime, m_A_Move_Phase_Shift + 2 * PI / m_A_Move_PeriodTime * m_SSP_Time_Start_R + PI, -m_A_Move_Amplitude, -m_A_Move_Amplitude_Shift);
        pelvis_offset_l = 0;
        pelvis_offset_r = 0;
    }

    a_move_l = 0;
    b_move_l = 0;
    a_move_r = 0;
    b_move_r = 0;

    ep[0] = x_swap + x_move_r + m_X_Offset;
    ep[1] = y_swap + y_move_r - m_Y_Offset / 2;
    ep[2] = z_swap + z_move_r + m_Z_Offset;
    ep[3] = a_swap + a_move_r - m_R_Offset / 2;
    ep[4] = b_swap + b_move_r + m_P_Offset;
    ep[5] = c_swap + c_move_r - m_A_Offset / 2;
    ep[6] = x_swap + x_move_l + m_X_Offset;
    ep[7] = y_swap + y_move_l + m_Y_Offset / 2;
    ep[8] = z_swap + z_move_l + m_Z_Offset;
    ep[9] = a_swap + a_move_l + m_R_Offset / 2;
    ep[10] = b_swap + b_move_l + m_P_Offset;
    ep[11] = c_swap + c_move_l + m_A_Offset / 2;

    // Compute body swing
    if(m_Time <= m_SSP_Time_End_L)
    {
        m_Body_Swing_Y = -ep[7];
        m_Body_Swing_Z = ep[8];
    }
    else
    {
        m_Body_Swing_Y = -ep[1];
        m_Body_Swing_Z = ep[2];
    }
	m_Body_Swing_Z -= Kinematics::LEG_LENGTH;

    // Compute arm swing
    if(m_X_Move_Amplitude == 0)
    {
        angle[12] = 0; // Right
        angle[13] = 0; // Left
    }
    else
    {
        angle[12] = wsin(m_Time, m_PeriodTime, PI * 1.5, -m_X_Move_Amplitude * m_Arm_Swing_Gain, 0);
        angle[13] = wsin(m_Time, m_PeriodTime, PI * 1.5, m_X_Move_Amplitude * m_Arm_Swing_Gain, 0);
    }

    if(m_Real_Running == true)
    {
        m_Time += TIME_UNIT;
        if(m_Time >= m_PeriodTime)
            m_Time = 0;
    }

    // Compute angles
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

    // Compute motor value
    for(int i=0; i<14; i++)
    {
		offset = (double)dir[i] * angle[i] * (g_pro54->MAX_VALUE)/180.0;
        if(i == 1) // R_HIP_ROLL
            offset += (double)dir[i] * pelvis_offset_r;
        else if(i == 7) // L_HIP_ROLL
            offset += (double)dir[i] * pelvis_offset_l;
        else if(i == 2 || i == 8) // R_HIP_PITCH or L_HIP_PITCH
            offset -= (double)dir[i] * HIP_PITCH_OFFSET * (g_pro54->MAX_VALUE)/180.0;

        outValue[i] = g_pro54->Angle2Value(initAngle[i]) + (int)offset;
    }


    // adjust balance offset
    if(BALANCE_ENABLE == true)
    {
		double rlGyroErr = MotionStatus::RL_GYRO;
		double fbGyroErr = MotionStatus::FB_GYRO;


        printf("Gyro FB:%f\n",fbGyroErr);


		outValue[1] += (int)(dir[1] * rlGyroErr * BALANCE_HIP_ROLL_GAIN*490.23); // R_HIP_ROLL
        outValue[7] += (int)(dir[7] * rlGyroErr * BALANCE_HIP_ROLL_GAIN*490.23); // L_HIP_ROLL

		outValue[5] -= (int)(dir[5] * rlGyroErr * BALANCE_ANKLE_ROLL_GAIN*490.23); // R_ANKLE_ROLL
        outValue[11] -= (int)(dir[11] * rlGyroErr * BALANCE_ANKLE_ROLL_GAIN*490.23); // L_ANKLE_ROLL

        outValue[3] -= (int)(dir[3] * fbGyroErr * BALANCE_KNEE_GAIN*490.23); // R_KNEE
        outValue[9] -= (int)(dir[9] * fbGyroErr * BALANCE_KNEE_GAIN*490.23); // L_KNEE

		outValue[4] -= (int)(dir[4] * fbGyroErr * BALANCE_ANKLE_PITCH_GAIN*490.23); // R_ANKLE_PITCH
        outValue[10] -= (int)(dir[10] * fbGyroErr * BALANCE_ANKLE_PITCH_GAIN*490.23); // L_ANKLE_PITCH

        printf("AnklePitch:%d\n",outValue[4]);
    }

    //fprintf(stderr,"%d, %d\n" ,outValue[1], outValue[5]);

    int id = -1;
    for(unsigned int jointIndex = 0; jointIndex < m_RobotInfo.size(); jointIndex++)
    {
    	id = m_RobotInfo[jointIndex].m_ID;

    	//not be fixed code
    	if(id >= 15 && id <=26)
    	{
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
    	}
    	else if( id == 1)
    		m_RobotInfo[jointIndex].m_Value = outValue[12];
    	else if(id == 2)
    		m_RobotInfo[jointIndex].m_Value = outValue[13];
//    	else if(id == 27)
//    		m_RobotInfo[jointIndex].m_Value = A_MOVE_AMPLITUDE;
    	else
    		continue;
    }
}




