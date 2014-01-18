//#include "RecursiveWalking.h"
#include "framework/motion/modules/recursive_walking.h"
#ifdef WEBOT_SIMULATION
#include "Vector.h"
#include "Matrix.h"
#include "Kinematics.h"
#include "Dynamics.h"
#else
#include "framework/math/vector.h"
#include "framework/math/matrix.h"
#include "framework/motion/kinematics.h"
#include "framework/motion/dynamics.h"
#include "framework/motion/PRO54.h"
//#include "framework/motion/modules/preview_control_walking.h
#endif

#ifdef WEBOT_SIMULATION
using namespace Robot;
#else
using namespace Thor;
#endif

#define PI (3.141592653589793)
#define G  (9810) // mm/s^2(= 9.81m/s^2 *1000mm/1m)

#ifdef REAL_ROBOT
#define FT_SENSOR_WINDOW_SIZE 3
static PRO54 *g_pro54 = new PRO54();
#endif

RecursiveWalking* RecursiveWalking::m_UniqueInstance = new RecursiveWalking();
//FILE *fp;
RecursiveWalking::RecursiveWalking()
{
	//std::vector<StepData> m_StepData;
#ifdef REAL_ROBOT
	uID = "RecursiveWalking";
	BALANCE_ENABLE = false;
#endif

	m_Real_Running = false; m_Ctrl_Running = false;
	m_IsCompleteCalcPattern = false;

	m_PresentGtoRightFootPosition = { 0, -105,   0, 0, 0, 0};
	m_PresentGtoLeftFootPosition  = { 0,  105,   0, 0, 0, 0};
	m_PresentGtoBodyPosition      = { 0,   0, 650, 0, 0, 0};

	m_ReferenceGtoBodyPosition 	    = { 0,   0, 650, 0, 0, 0};
	m_ReferenceGtoRightFootPosition = { 0, -105,   0, 0, 0, 0};
	m_ReferenceGtoLeftFootPosition  = { 0,  105,   0, 0, 0, 0};


	m_Time = 0; m_WalkingTime = 0; m_ReferenceTime = 0;
	m_PresentStepNum = 0;
	m_PreviousStepNum = 0;

	m_PeriodTime = 848;
	m_DSP_Ratio = 0.2;
	m_SSP_Ratio = 1 - m_DSP_Ratio;
	m_Foot_Move_PeriodTime = m_SSP_Ratio*m_PeriodTime;
	m_Body_Move_PeriodTime = m_PeriodTime;

	m_SSP_Time = m_Foot_Move_PeriodTime;
	m_SSP_Time_Start = m_DSP_Ratio*m_PeriodTime/2.0;
	m_SSP_Time_End = (1 + m_SSP_Ratio)*m_PeriodTime / 2.0;


	//Initial Pose
	X_Offset = 0.0; Y_OFfset = 56.0; Z_Offset = Kinematics::LEG_LENGTH - 650.0;
	A_Offset = 0.0;	B_Offset = 0.0;  C_Offset = 0.0;

	m_X_ZMP_Init = -14.251206; m_X_ZMP_CenterShift = 0.0;
	m_Y_ZMP_Convergence = 0.0; m_Y_ZMP_CenterShift = 0.0;

	//for Balance
	HIP_PITCH_OFFSET = 0.0;

	P_GAIN = 64;
	I_GAIN = 0;
	D_GAIN = 0;

	matCOBtoRH.resize(4,4);
	matCOBtoRH = GetTranslationMatrix(0, -Kinematics::LEG_SIDE_OFFSET*0.5, 0);
	matCOBtoLH.resize(4,4);
	matCOBtoLH = GetTranslationMatrix(0,  Kinematics::LEG_SIDE_OFFSET*0.5, 0);

	matRHtoCOB.resize(4,4);
	matRHtoCOB = GetTranslationMatrix(0,  Kinematics::LEG_SIDE_OFFSET*0.5, 0);
	matLHtoCOB.resize(4,4);
	matLHtoCOB = GetTranslationMatrix(0, -Kinematics::LEG_SIDE_OFFSET*0.5, 0);

	dir[0] = -1;  dir[1] = -1; dir[2] =  1;   dir[3] = 1;  dir[4] = -1; dir[5] =  1;
	dir[6] = -1;  dir[7] = -1; dir[8] = -1;   dir[9] = -1; dir[10] = 1; dir[11] = 1;
	dir[12] = -1; dir[13] = 1; dir[14] = -1;  dir[15] = 1;

#ifdef WEBOT_SIMULATION
	//for webot
	dir_output[0] = -1; dir_output[1] = -1; dir_output[2] =  1; dir_output[3] =  1; dir_output[4] = -1; dir_output[5] = 1;
	dir_output[6] = -1; dir_output[7] = -1; dir_output[8] = -1; dir_output[9] = -1; dir_output[10]=  1; dir_output[11] = 1;
	dir_output[12] = -1; dir_output[13] = 1;  dir_output[14] = -1; dir_output[15] =  1;
	InitAngle[0]  =   0.0;  InitAngle[1]  =  0.0;  InitAngle[2]  = -5.7106;  InitAngle[3] =  11.4212; InitAngle[4]  =  5.7106; InitAngle[5]  = 0.0;
	InitAngle[6]  =   0.0;  InitAngle[7]  =  0.0;  InitAngle[8]  =  5.7106;  InitAngle[9] = -11.4212; InitAngle[10] = -5.7106; InitAngle[11] = 0.0;
	InitAngle[12] = -45.0,  InitAngle[13] = 45.0;  InitAngle[14] =  45.0;    InitAngle[15] =  -45.0;
	//	dir_output = {  -1,    -1,        1,        1,      -1,     1,    -1,    -1,       -1,         -1,         1,     1,     -1,      1,    -1,     1};
	//	InitAngle  = { 0.0,   0.0,  -5.7106,  11.4212,  5.7106,   0.0,   0.0,   0.0,   5.7106,   -11.4212,   -5.7106,   0.0,  -45.0,   45.0,  45.0,  -45.0};
#else
	//for thor
	dir_output[0] = -1; dir_output[1] = -1; dir_output[2] = -1; dir_output[3] = -1; dir_output[4] =  1; dir_output[5] = 1;
	dir_output[6] = -1; dir_output[7] = -1; dir_output[8] =  1; dir_output[9] =  1; dir_output[10]= -1; dir_output[11] = 1;
	dir_output[12] = -1; dir_output[13] = 1;  dir_output[14] = -1; dir_output[15] =  1;
	InitAngle[0]  =   0.0;  InitAngle[1]  =  0.0;  InitAngle[2]  =  5.7106;  InitAngle[3] =  33.5788; InitAngle[4]  = -5.7106; InitAngle[5]  = 0.0;
	InitAngle[6]  =   0.0;  InitAngle[7]  =  0.0;  InitAngle[8]  = -5.7106;  InitAngle[9] = -33.5788; InitAngle[10] =  5.7106; InitAngle[11] = 0.0;
	InitAngle[12] = -45.0,  InitAngle[13] = 45.0;  InitAngle[14] =  45.0;    InitAngle[15] =  -45.0;
#endif

	r_arm[0] = -45.0; r_arm[1] =  0.17*180.0/PI - 90.0; r_arm[2] =  90.0; r_arm[3] =  90.0; r_arm[4] = 0.0; r_arm[5] = 0.0;
	l_arm[0] =  45.0; l_arm[1] = -0.17*180.0/PI + 90.0; l_arm[2] = -90.0; l_arm[3] = -90.0; l_arm[4] = 0.0; l_arm[5] = 0.0;

	for(int i=0; i<6; i++)
	{
		r_arm_init[i] = r_arm[i];
		l_arm_init[i] = l_arm[i];
	}

	P_GAIN = 64;
	I_GAIN = 0;
	D_GAIN = 0;

	DEBUG_PRINT = false;

	BALANCE_ENABLE = true;
	HIP_PITCH_OFFSET = 0.0;

#ifdef REAL_ROBOT
	m_right_leg_ft_fz_N_array = 0;
	m_left_leg_ft_fz_N_array = 0;
	BALANCE_ENABLE = true;
	BALANCE_KNEE_GAIN = 0.300000;
	BALANCE_ANKLE_PITCH_GAIN = 0.900000;
	BALANCE_HIP_ROLL_GAIN = 0.500000;
	BALANCE_ANKLE_ROLL_GAIN = 1.000000;

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

	m_iu_roll_init_rad = 0;
	m_iu_pitch_init_rad = 0;
	m_iu_yaw_init_rad = 0;

	m_right_leg_ft_fz_N_array = 0;
	m_left_leg_ft_fz_N_array = 0;

#endif
	m_PatternDataSize = 0;
	m_play_idx = 0;
	m_dplay_idx =0;

	//fp = fopen("aaaaaaa.txt", "w");
}

RecursiveWalking::~RecursiveWalking()
{   }

void RecursiveWalking::Initialize()
{
#ifdef REAL_ROBOT
	//////Initialize parameter for control dxl_pro
	if(MotionStatus::m_CurrentJoints.size() != 0)
		m_RobotInfo = MotionStatus::m_CurrentJoints;
	else
	{
		fprintf(stderr,"MotionStatus is not initialized");
		return;
	}


	if(	m_right_leg_ft_fz_N_array != 0)
		delete[] m_right_leg_ft_fz_N_array;
	if(m_left_leg_ft_fz_N_array != 0)
		delete[] m_left_leg_ft_fz_N_array;

	m_right_leg_ft_fz_N_array = new double[FT_SENSOR_WINDOW_SIZE];
	m_left_leg_ft_fz_N_array = new double[FT_SENSOR_WINDOW_SIZE];

#endif

	matGtoCOB = GetTransformMatrix(m_PresentGtoBodyPosition.x, m_PresentGtoBodyPosition.y, m_PresentGtoBodyPosition.z, m_PresentGtoBodyPosition.roll, m_PresentGtoBodyPosition.pitch, m_PresentGtoBodyPosition.yaw );
	matGtoRF = GetTransformMatrix(m_PresentGtoRightFootPosition.x, m_PresentGtoRightFootPosition.y, m_PresentGtoRightFootPosition.z, m_PresentGtoRightFootPosition.roll, m_PresentGtoRightFootPosition.pitch, m_PresentGtoRightFootPosition.yaw);
	matGtoLF = GetTransformMatrix(m_PresentGtoLeftFootPosition.x, m_PresentGtoLeftFootPosition.y, m_PresentGtoLeftFootPosition.z, m_PresentGtoLeftFootPosition.roll, m_PresentGtoLeftFootPosition.pitch, m_PresentGtoLeftFootPosition.yaw);

	matCOBtoG = GetTransformMatrixInverse(matGtoCOB);
	matRHtoRF = matRHtoCOB*matCOBtoG*matGtoRF;
	matLHtoLF = matLHtoCOB*matCOBtoG*matGtoLF;

	Pose3D epr, epl;

	epr = GetPose3DfromTransformMatrix(matRHtoRF);
	epl = GetPose3DfromTransformMatrix(matLHtoLF);

	double angle[12];
	if(computeIK(&angle[0], epr.x, epr.y, epr.z+Kinematics::LEG_LENGTH, epr.roll, epr.pitch, epr.yaw) == false) {
		printf("IKsolve failed\n");
		return;
	}

	if(computeIK(&angle[6], epl.x, epl.y, epl.z+Kinematics::LEG_LENGTH, epl.roll, epl.pitch, epl.yaw) == false) {
		printf("IKsolve failed\n");
		return;
	}

	for(int idx = 0; idx < 6; idx++)
	{
		angle[idx] = (double)dir_output[idx]*angle[idx]*180.0/PI + InitAngle[idx];
		angle[idx+6] = (double)dir_output[idx+6]*angle[idx+6]*180.0/PI + InitAngle[idx+6];
	}
#ifdef WEBOT_SIMULATION
	for(int idx = 0; idx < 12; idx++) {
		Webot_Rad[idx] = angle[idx]*PI/180.0;
	}
#else
	for(int idx = 0; idx < 16; idx++)
	{
		//		double offset = (double)dir[i] * angle[i] * (g_pro54->MAX_VALUE)/180.0;
		//        outValue[i] = g_pro54->Angle2Value(initAngle[i]) + (int)offset;
		outValue[idx] = g_pro54->Angle2Value(angle[idx]);
	}

	outValue[2] -= (double)dir_output[2] * HIP_PITCH_OFFSET * (g_pro54->MAX_VALUE)/180.0;
	outValue[8] -= (double)dir_output[8] * HIP_PITCH_OFFSET * (g_pro54->MAX_VALUE)/180.0;

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
		else if( id == 17) {
			m_RobotInfo[jointIndex].m_Value = outValue[1];
			m_RobotInfo[jointIndex].m_Pgain = P_GAIN;
			m_RobotInfo[jointIndex].m_Igain = I_GAIN;
			m_RobotInfo[jointIndex].m_Dgain = D_GAIN;
		}
		else if( id == 19) {
			m_RobotInfo[jointIndex].m_Value = outValue[2];
			m_RobotInfo[jointIndex].m_Pgain = P_GAIN;
			m_RobotInfo[jointIndex].m_Igain = I_GAIN;
			m_RobotInfo[jointIndex].m_Dgain = D_GAIN;
		}
		else if(id == 21) {
			m_RobotInfo[jointIndex].m_Value = outValue[3];
			m_RobotInfo[jointIndex].m_Pgain = P_GAIN;
			m_RobotInfo[jointIndex].m_Igain = I_GAIN;
			m_RobotInfo[jointIndex].m_Dgain = D_GAIN;
		}
		else if(id == 23) {
			m_RobotInfo[jointIndex].m_Value = outValue[4];
			m_RobotInfo[jointIndex].m_Pgain = P_GAIN;
			m_RobotInfo[jointIndex].m_Igain = I_GAIN;
			m_RobotInfo[jointIndex].m_Dgain = D_GAIN;
		}
		else if(id == 25) {
			m_RobotInfo[jointIndex].m_Value = outValue[5];
			m_RobotInfo[jointIndex].m_Pgain = P_GAIN;
			m_RobotInfo[jointIndex].m_Igain = I_GAIN;
			m_RobotInfo[jointIndex].m_Dgain = D_GAIN;
		}
		else if(id == 16) {
			m_RobotInfo[jointIndex].m_Value = outValue[6];
			m_RobotInfo[jointIndex].m_Pgain = P_GAIN;
			m_RobotInfo[jointIndex].m_Igain = I_GAIN;
			m_RobotInfo[jointIndex].m_Dgain = D_GAIN;
		}
		else if( id == 18) {
			m_RobotInfo[jointIndex].m_Value = outValue[7];
			m_RobotInfo[jointIndex].m_Pgain = P_GAIN;
			m_RobotInfo[jointIndex].m_Igain = I_GAIN;
			m_RobotInfo[jointIndex].m_Dgain = D_GAIN;
		}
		else if( id == 20) {
			m_RobotInfo[jointIndex].m_Value = outValue[8];
			m_RobotInfo[jointIndex].m_Pgain = P_GAIN;
			m_RobotInfo[jointIndex].m_Igain = I_GAIN;
			m_RobotInfo[jointIndex].m_Dgain = D_GAIN;
		}
		else if(id == 22) {
			m_RobotInfo[jointIndex].m_Value = outValue[9];
			m_RobotInfo[jointIndex].m_Pgain = P_GAIN;
			m_RobotInfo[jointIndex].m_Igain = I_GAIN;
			m_RobotInfo[jointIndex].m_Dgain = D_GAIN;
		}
		else if(id == 24) {
			m_RobotInfo[jointIndex].m_Value = outValue[10];
			m_RobotInfo[jointIndex].m_Pgain = P_GAIN;
			m_RobotInfo[jointIndex].m_Igain = I_GAIN;
			m_RobotInfo[jointIndex].m_Dgain = D_GAIN;
		}
		else if(id == 26) {
			m_RobotInfo[jointIndex].m_Value = outValue[11];
			m_RobotInfo[jointIndex].m_Pgain = P_GAIN;
			m_RobotInfo[jointIndex].m_Igain = I_GAIN;
			m_RobotInfo[jointIndex].m_Dgain = D_GAIN;
		}
//		else if(id == 1) {
//			m_RobotInfo[jointIndex].m_Value = outValue[12];
//			m_RobotInfo[jointIndex].m_Pgain = P_GAIN;
//			m_RobotInfo[jointIndex].m_Igain = I_GAIN;
//			m_RobotInfo[jointIndex].m_Dgain = D_GAIN;
//		}
//		else if(id == 2) {
//			m_RobotInfo[jointIndex].m_Value = outValue[13];
//			m_RobotInfo[jointIndex].m_Pgain = P_GAIN;
//			m_RobotInfo[jointIndex].m_Igain = I_GAIN;
//			m_RobotInfo[jointIndex].m_Dgain = D_GAIN;
//		}
//		else if(id == 7) {
//			m_RobotInfo[jointIndex].m_Value = outValue[14];
//			m_RobotInfo[jointIndex].m_Pgain = P_GAIN;
//			m_RobotInfo[jointIndex].m_Igain = I_GAIN;
//			m_RobotInfo[jointIndex].m_Dgain = D_GAIN;
//		}
//		else if(id == 8) {
//			m_RobotInfo[jointIndex].m_Value = outValue[15];
//			m_RobotInfo[jointIndex].m_Pgain = P_GAIN;
//			m_RobotInfo[jointIndex].m_Igain = I_GAIN;
//			m_RobotInfo[jointIndex].m_Dgain = D_GAIN;
//		}
		else
			continue;
	}
#endif
}

bool RecursiveWalking::IsRunning()
{
	return m_Real_Running;
}

void RecursiveWalking::Start()
{
#ifdef REAL_ROBOT
	if(MotionStatus::m_CurrentJoints.size() != 0)
		m_RobotInfo = MotionStatus::m_CurrentJoints;
	else
	{
		fprintf(stderr,"MotionStatus is not initialized");
		return;
	}
#endif

//	if(m_IsCompleteCalcPattern)
//	{
//		m_Ctrl_Running = true;
//		m_Real_Running = true;
//	}
//	else
//		printf("pattern calc is not completed\n");

	m_Ctrl_Running = true;
	if(m_IsCompleteCalcPattern == true)
		m_Real_Running = true;
	else
		m_Real_Running = false;

	m_play_idx = 0;
	m_dplay_idx = 0;
}

void RecursiveWalking::Stop()
{
	m_Real_Running = false;
}

void RecursiveWalking::AddStepData(StepData step_data)
{
	m_StepData.push_back(step_data);
}

void RecursiveWalking::ClearStepData()
{
	if(m_StepData.size() != 0)
		m_StepData.clear();
}

void RecursiveWalking::GetPresentReferencePose(Pose3D *present_body, Pose3D *present_right_foot, Pose3D *present_left_foot)
{
	present_body->x = m_ReferenceGtoBodyPosition.x;	      present_body->y = m_ReferenceGtoBodyPosition.y;         present_body->z = m_ReferenceGtoBodyPosition.z;
	present_body->roll = m_ReferenceGtoBodyPosition.roll; present_body->pitch = m_ReferenceGtoBodyPosition.pitch; present_body->yaw = m_ReferenceGtoBodyPosition.yaw;

	present_right_foot->x = m_ReferenceGtoRightFootPosition.x;	     present_right_foot->y = m_ReferenceGtoRightFootPosition.y;         present_right_foot->z = m_ReferenceGtoRightFootPosition.z;
	present_right_foot->roll = m_ReferenceGtoRightFootPosition.roll; present_right_foot->pitch = m_ReferenceGtoRightFootPosition.pitch; present_right_foot->yaw = m_ReferenceGtoRightFootPosition.yaw;

	present_left_foot->x = m_ReferenceGtoLeftFootPosition.x;	   present_left_foot->y = m_ReferenceGtoLeftFootPosition.y;         present_left_foot->z = m_ReferenceGtoLeftFootPosition.z;
	present_left_foot->roll = m_ReferenceGtoLeftFootPosition.roll; present_left_foot->pitch = m_ReferenceGtoLeftFootPosition.pitch; present_left_foot->yaw = m_ReferenceGtoLeftFootPosition.yaw;
}


void RecursiveWalking::SetInitAngleinRad(double roll_init_rad, double pitch_init_rad)
{
#ifdef REAL_ROBOT
	m_iu_roll_init_rad = roll_init_rad;
	m_iu_pitch_init_rad = pitch_init_rad;
#endif
}

void RecursiveWalking::SetRefZMPDecisionParameter(double X_ZMP_CenterShift, double Y_ZMP_CenterShift, double Y_ZMP_Convergence)
{
	m_X_ZMP_CenterShift = X_ZMP_CenterShift;
	m_Y_ZMP_CenterShift = Y_ZMP_CenterShift;
	m_Y_ZMP_Convergence = Y_ZMP_Convergence;
}

double RecursiveWalking::wsin(double time, double period, double period_shift, double mag, double mag_shift)
{
	return mag * sin(2 * PI / period * time - period_shift) + mag_shift;
}

double RecursiveWalking::wsigmoid(double time, double period, double time_shift, double mag, double mag_shift, double sigmoid_ratio, double distortion_ratio)
{
	double value = mag_shift, Amplitude = 0.0, sigmoid_distor_gain = 1.0, t = 0.0;
	if (sigmoid_ratio >= 1) {

		if( time >= time_shift+period*(2-sigmoid_ratio)) {
			value = mag_shift + mag;
		}
		else
		{
			t = 2.0*PI*(time - time_shift)/(period*(2-sigmoid_ratio));
			sigmoid_distor_gain = distortion_ratio + (1-distortion_ratio)*(time-(time_shift+period*(1-sigmoid_ratio)))/(period*(2-sigmoid_ratio));
			Amplitude = mag/(2.0*PI);
			value = mag_shift + Amplitude*(t - sigmoid_distor_gain*sin(t));
		}
	}
	else if( sigmoid_ratio < 1) {
		if( time <= time_shift+period*(1-sigmoid_ratio))
			value = mag_shift;
		else {
			t = 2.0*PI*(time - time_shift-period*(1-sigmoid_ratio))/(period*sigmoid_ratio);
			sigmoid_distor_gain = distortion_ratio + (1-distortion_ratio)*(time-(time_shift+period*(1-sigmoid_ratio)))/(period*sigmoid_ratio);
			Amplitude = mag/(2.0*PI);
			value = mag_shift + Amplitude*(t - sigmoid_distor_gain*sin(t));
		}
	}

	return value;
}

void RecursiveWalking::CalcIdxData()
{
	int time = 0;
	int step_idx = 0 ;
	m_StepIdxData(0) = 0;
	for(int idx = 1; idx <m_PatternDataSize ; idx++)
	{
		for(step_idx = m_StepIdxData(idx-1); step_idx < m_StepData.size(); step_idx++)
		{
			if(m_StepData[step_idx].TimeData.dAbsStepTime >= time + idx*TIME_UNIT)
			{
				break;
			}

		}
		m_StepIdxData(idx) = step_idx;
	}
}

void RecursiveWalking::CalcDetailTimeIdx()
{
	vecd _temp;
	int size = m_DetailTimeIdx.size();
	_temp.resize(size);

	_temp(0) = m_StepData[0].TimeData.dAbsStepTime;
	_temp(size-1) = m_StepData[m_StepData.size() - 1].TimeData.dAbsStepTime;

	for(int idx=1; idx <= m_StepData.size() - 2; idx++)
	{
		double ref_time = m_StepData[idx-1].TimeData.dAbsStepTime;
		double end_time = m_StepData[idx].TimeData.dAbsStepTime;
		double period_time = end_time - ref_time;
		double dsp_ratio = m_StepData[idx].TimeData.dDSPratio;
		double ssp_ratio = 1 - dsp_ratio;
		double ssp_start_time = ref_time + period_time*dsp_ratio*0.5;
		double ssp_end_time = ref_time + (1 + ssp_ratio)*period_time*0.5;

		_temp(idx*3-2) = ssp_start_time;
		_temp(idx*3-1) = ssp_end_time;
		_temp(idx*3-0) = end_time;
	}

	for(int idx = 0 ; idx < _temp.size(); idx++)
	{
		m_DetailTimeIdx(idx) = (int)(_temp(idx)/(double)TIME_UNIT);
	}
}

void RecursiveWalking::CalcRefZMP()
{
	int time = 0;
	int step_idx = 0;

	for(int idx = 0; idx < m_PatternDataSize; idx++)
	{
		step_idx = m_StepIdxData(idx);
		if(m_StepData[step_idx].TimeData.bWalkingState == InWalkingStarting)
		{
			m_ZMP_Reference_X(idx, 0) = (m_StepData[step_idx].PositionData.stRightFootPosition.x + m_StepData[step_idx].PositionData.stLeftFootPosition.x)*0.5 + m_X_ZMP_Init;
			m_ZMP_Reference_Y(idx, 0) = (m_StepData[step_idx].PositionData.stRightFootPosition.y + m_StepData[step_idx].PositionData.stLeftFootPosition.y)*0.5;
		}
		else if(m_StepData[step_idx].TimeData.bWalkingState == InWalking)
		{
			if(m_StepData[step_idx].PositionData.bMovingFoot == RFootMove)
			{
				m_ZMP_Reference_X(idx, 0) = m_StepData[step_idx].PositionData.stLeftFootPosition.x + m_X_ZMP_CenterShift;
				m_ZMP_Reference_Y(idx, 0) = m_StepData[step_idx].PositionData.stLeftFootPosition.y + m_Y_ZMP_CenterShift - m_Y_ZMP_Convergence;
			}
			else if(m_StepData[step_idx].PositionData.bMovingFoot == LFootMove)
			{
				m_ZMP_Reference_X(idx, 0) = m_StepData[step_idx].PositionData.stRightFootPosition.x + m_X_ZMP_CenterShift;
				m_ZMP_Reference_Y(idx, 0) = m_StepData[step_idx].PositionData.stRightFootPosition.y + m_Y_ZMP_CenterShift + m_Y_ZMP_Convergence;
			}
			else
			{
				m_ZMP_Reference_X(idx, 0) = (m_StepData[step_idx].PositionData.stRightFootPosition.x + m_StepData[step_idx].PositionData.stLeftFootPosition.x)*0.5 + m_X_ZMP_CenterShift;
				m_ZMP_Reference_Y(idx, 0) = (m_StepData[step_idx].PositionData.stRightFootPosition.y + m_StepData[step_idx].PositionData.stLeftFootPosition.y)*0.5 + m_Y_ZMP_CenterShift;
			}
		}
		else if(m_StepData[step_idx].TimeData.bWalkingState == InWalkingEnding)
		{
			m_ZMP_Reference_X(idx, 0) = (m_StepData[step_idx].PositionData.stRightFootPosition.x + m_StepData[step_idx].PositionData.stLeftFootPosition.x)*0.5 + m_X_ZMP_Init;
			m_ZMP_Reference_Y(idx, 0) = (m_StepData[step_idx].PositionData.stRightFootPosition.y + m_StepData[step_idx].PositionData.stLeftFootPosition.y)*0.5;
		}
	}
}

void RecursiveWalking::CalcEndPointData()
{
	double time = 0;
	double ref_time = 0;
	int step_idx = 0;

	double period_time, dsp_ratio, ssp_ratio, foot_move_period_time, body_move_period_time, ssp_time_start, ssp_time_end;
	double x_move_amp, y_move_amp, z_move_amp, a_move_amp, b_move_amp, c_move_amp, z_vibe_amp;
	double x_move_amp_shift, y_move_amp_shift, z_move_amp_shift, a_move_amp_shift, b_move_amp_shift, c_move_amp_shift, z_vibe_amp_shift;
	double z_vibe_phase_shift;
	double x_move, y_move, z_move, a_move, b_move, c_move, z_vibe;

	//printf("%f\n", m_StepData[0].PositionData.stRightFootPosition.pitch);
	for(int ep_idx = 0; ep_idx < m_DetailTimeIdx(0); ep_idx++)
	{
		m_matvGtoRF[ep_idx] = GetTransformMatrix(m_StepData[step_idx].PositionData.stRightFootPosition.x, m_StepData[step_idx].PositionData.stRightFootPosition.y, m_StepData[step_idx].PositionData.stRightFootPosition.z,
												 m_StepData[step_idx].PositionData.stRightFootPosition.roll, m_StepData[step_idx].PositionData.stRightFootPosition.pitch, m_StepData[step_idx].PositionData.stRightFootPosition.yaw);
		m_matvGtoLF[ep_idx] = GetTransformMatrix(m_StepData[step_idx].PositionData.stLeftFootPosition.x, m_StepData[step_idx].PositionData.stLeftFootPosition.y, m_StepData[step_idx].PositionData.stLeftFootPosition.z,
												 m_StepData[step_idx].PositionData.stLeftFootPosition.roll, m_StepData[step_idx].PositionData.stLeftFootPosition.pitch, m_StepData[step_idx].PositionData.stLeftFootPosition.yaw);

		m_Balancing_Idx(ep_idx) = 0;
	}

	for(step_idx = 1; step_idx < m_StepData.size() - 1; step_idx++)
	{
		ref_time = m_StepData[step_idx-1].TimeData.dAbsStepTime;
		dsp_ratio = m_StepData[step_idx].TimeData.dDSPratio;
		ssp_ratio = 1-dsp_ratio;
		period_time = m_StepData[step_idx].TimeData.dAbsStepTime - ref_time;
		body_move_period_time = period_time;
		foot_move_period_time = period_time*ssp_ratio;
		ssp_time_start = period_time*dsp_ratio*0.5;
		ssp_time_end = period_time*(1+ssp_ratio)*0.5;

		if(m_StepData[step_idx].PositionData.bMovingFoot == RFootMove)
		{
			x_move_amp = (m_StepData[step_idx].PositionData.stRightFootPosition.x - m_StepData[step_idx-1].PositionData.stRightFootPosition.x);
			x_move_amp_shift = m_StepData[step_idx-1].PositionData.stRightFootPosition.x;

			y_move_amp = (m_StepData[step_idx].PositionData.stRightFootPosition.y - m_StepData[step_idx-1].PositionData.stRightFootPosition.y);
			y_move_amp_shift = m_StepData[step_idx-1].PositionData.stRightFootPosition.y;

			z_move_amp = (m_StepData[step_idx].PositionData.stRightFootPosition.z - m_StepData[step_idx-1].PositionData.stRightFootPosition.z);
			z_move_amp_shift = m_StepData[step_idx-1].PositionData.stRightFootPosition.z;

			a_move_amp = (m_StepData[step_idx].PositionData.stRightFootPosition.roll - m_StepData[step_idx-1].PositionData.stRightFootPosition.roll);
			a_move_amp_shift = m_StepData[step_idx-1].PositionData.stRightFootPosition.roll;

			b_move_amp = (m_StepData[step_idx].PositionData.stRightFootPosition.pitch - m_StepData[step_idx-1].PositionData.stRightFootPosition.pitch);
			b_move_amp_shift = m_StepData[step_idx-1].PositionData.stRightFootPosition.pitch;

			c_move_amp = (m_StepData[step_idx].PositionData.stRightFootPosition.yaw - m_StepData[step_idx-1].PositionData.stRightFootPosition.yaw);
			c_move_amp_shift = m_StepData[step_idx-1].PositionData.stRightFootPosition.yaw;

			z_vibe_amp = m_StepData[step_idx].PositionData.dFootHeight * 0.5;
			z_vibe_amp_shift = z_vibe_amp;
			z_vibe_phase_shift = PI*0.5;
		}
		else if(m_StepData[step_idx].PositionData.bMovingFoot == LFootMove)	{
			x_move_amp = (m_StepData[step_idx].PositionData.stLeftFootPosition.x - m_StepData[step_idx-1].PositionData.stLeftFootPosition.x);
			x_move_amp_shift = m_StepData[step_idx-1].PositionData.stLeftFootPosition.x;

			y_move_amp = (m_StepData[step_idx].PositionData.stLeftFootPosition.y - m_StepData[step_idx-1].PositionData.stLeftFootPosition.y);
			y_move_amp_shift = m_StepData[step_idx-1].PositionData.stLeftFootPosition.y;

			z_move_amp = (m_StepData[step_idx].PositionData.stLeftFootPosition.z - m_StepData[step_idx-1].PositionData.stLeftFootPosition.z);
			z_move_amp_shift = m_StepData[step_idx-1].PositionData.stLeftFootPosition.z;

			a_move_amp = (m_StepData[step_idx].PositionData.stLeftFootPosition.roll - m_StepData[step_idx-1].PositionData.stLeftFootPosition.roll);
			a_move_amp_shift = m_StepData[step_idx-1].PositionData.stLeftFootPosition.roll;

			b_move_amp = (m_StepData[step_idx].PositionData.stLeftFootPosition.pitch - m_StepData[step_idx-1].PositionData.stLeftFootPosition.pitch);
			b_move_amp_shift = m_StepData[step_idx-1].PositionData.stLeftFootPosition.pitch;

			c_move_amp = (m_StepData[step_idx].PositionData.stLeftFootPosition.yaw - m_StepData[step_idx-1].PositionData.stLeftFootPosition.yaw);
			c_move_amp_shift = m_StepData[step_idx-1].PositionData.stLeftFootPosition.yaw;

			z_vibe_amp = m_StepData[step_idx].PositionData.dFootHeight * 0.5;
			z_vibe_amp_shift = z_vibe_amp;
			z_vibe_phase_shift = PI*0.5;
		}
		else {
			x_move_amp = 0.0;
			x_move_amp_shift = 0.0;

			y_move_amp = 0.0;
			y_move_amp_shift = 0.0;

			z_move_amp = 0.0;
			z_move_amp_shift = 0.0;

			a_move_amp = 0.0;
			a_move_amp_shift = 0.0;

			b_move_amp = 0.0;
			b_move_amp_shift = 0.0;

			c_move_amp = 0.0;
			c_move_amp_shift = 0.0;

			z_vibe_amp = 0.0;
			z_vibe_amp_shift = z_vibe_amp;
			z_vibe_phase_shift = PI*0.5;
		}


		//refer to detail_time_idx
		for(int ep_idx = m_DetailTimeIdx(step_idx*3-3); ep_idx < m_DetailTimeIdx(step_idx*3 -2); ep_idx++)
		{
			x_move = wsigmoid(ssp_time_start, foot_move_period_time, ssp_time_start, x_move_amp, x_move_amp_shift, m_StepData[step_idx].TimeData.sigmoid_ratio_x,     m_StepData[step_idx].TimeData.sigmoid_distortion_x);
			y_move = wsigmoid(ssp_time_start, foot_move_period_time, ssp_time_start, y_move_amp, y_move_amp_shift, m_StepData[step_idx].TimeData.sigmoid_ratio_y,     m_StepData[step_idx].TimeData.sigmoid_distortion_y);
			z_move = wsigmoid(ssp_time_start, foot_move_period_time, ssp_time_start, z_move_amp, z_move_amp_shift, m_StepData[step_idx].TimeData.sigmoid_ratio_z,     m_StepData[step_idx].TimeData.sigmoid_distortion_z);
			a_move = wsigmoid(ssp_time_start, foot_move_period_time, ssp_time_start, a_move_amp, a_move_amp_shift, m_StepData[step_idx].TimeData.sigmoid_ratio_roll,  m_StepData[step_idx].TimeData.sigmoid_distortion_roll);
			b_move = wsigmoid(ssp_time_start, foot_move_period_time, ssp_time_start, b_move_amp, b_move_amp_shift, m_StepData[step_idx].TimeData.sigmoid_ratio_pitch, m_StepData[step_idx].TimeData.sigmoid_distortion_pitch);
			c_move = wsigmoid(ssp_time_start, foot_move_period_time, ssp_time_start, c_move_amp, c_move_amp_shift, m_StepData[step_idx].TimeData.sigmoid_ratio_yaw,   m_StepData[step_idx].TimeData.sigmoid_distortion_yaw);
			z_vibe = wsin(ssp_time_start, foot_move_period_time, z_vibe_phase_shift + 2.0*PI*ssp_time_start/foot_move_period_time, z_vibe_amp, z_vibe_amp_shift);

			if(m_StepData[step_idx].PositionData.bMovingFoot == RFootMove)
			{
				m_matvGtoRF[ep_idx] = GetTransformMatrix(x_move, y_move, z_move + z_vibe, a_move, b_move, c_move);
				m_matvGtoLF[ep_idx] = GetTransformMatrix(m_StepData[step_idx].PositionData.stLeftFootPosition.x, m_StepData[step_idx].PositionData.stLeftFootPosition.y, m_StepData[step_idx].PositionData.stLeftFootPosition.z,
														 m_StepData[step_idx].PositionData.stLeftFootPosition.roll, m_StepData[step_idx].PositionData.stLeftFootPosition.pitch, m_StepData[step_idx].PositionData.stLeftFootPosition.yaw);

				m_Balancing_Idx(ep_idx) = 1;
			}
			else if(m_StepData[step_idx].PositionData.bMovingFoot == LFootMove)	{
				m_matvGtoRF[ep_idx] = GetTransformMatrix(m_StepData[step_idx].PositionData.stRightFootPosition.x, m_StepData[step_idx].PositionData.stRightFootPosition.y, m_StepData[step_idx].PositionData.stRightFootPosition.z,
														 m_StepData[step_idx].PositionData.stRightFootPosition.roll, m_StepData[step_idx].PositionData.stRightFootPosition.pitch, m_StepData[step_idx].PositionData.stRightFootPosition.yaw);
				m_matvGtoLF[ep_idx] = GetTransformMatrix(x_move, y_move, z_move + z_vibe, a_move, b_move, c_move);
				m_Balancing_Idx(ep_idx) = 5;
			}
			else {

				m_matvGtoRF[ep_idx] = GetTransformMatrix(m_StepData[step_idx].PositionData.stRightFootPosition.x, m_StepData[step_idx].PositionData.stRightFootPosition.y, m_StepData[step_idx].PositionData.stRightFootPosition.z,
														 m_StepData[step_idx].PositionData.stRightFootPosition.roll, m_StepData[step_idx].PositionData.stRightFootPosition.pitch, m_StepData[step_idx].PositionData.stRightFootPosition.yaw);
				m_matvGtoLF[ep_idx] = GetTransformMatrix(m_StepData[step_idx].PositionData.stLeftFootPosition.x, m_StepData[step_idx].PositionData.stLeftFootPosition.y, m_StepData[step_idx].PositionData.stLeftFootPosition.z,
														 m_StepData[step_idx].PositionData.stLeftFootPosition.roll, m_StepData[step_idx].PositionData.stLeftFootPosition.pitch, m_StepData[step_idx].PositionData.stLeftFootPosition.yaw);
				m_Balancing_Idx(ep_idx) = 0;
			}
		}

		for(int ep_idx = m_DetailTimeIdx(step_idx*3-2); ep_idx < m_DetailTimeIdx(step_idx*3-1); ep_idx++)
		{
			time = ep_idx*TIME_UNIT - ref_time;
			x_move = wsigmoid(time, foot_move_period_time, ssp_time_start, x_move_amp, x_move_amp_shift, m_StepData[step_idx].TimeData.sigmoid_ratio_x,     m_StepData[step_idx].TimeData.sigmoid_distortion_x);
			y_move = wsigmoid(time, foot_move_period_time, ssp_time_start, y_move_amp, y_move_amp_shift, m_StepData[step_idx].TimeData.sigmoid_ratio_y,     m_StepData[step_idx].TimeData.sigmoid_distortion_y);
			z_move = wsigmoid(time, foot_move_period_time, ssp_time_start, z_move_amp, z_move_amp_shift, m_StepData[step_idx].TimeData.sigmoid_ratio_z,     m_StepData[step_idx].TimeData.sigmoid_distortion_z);
			a_move = wsigmoid(time, foot_move_period_time, ssp_time_start, a_move_amp, a_move_amp_shift, m_StepData[step_idx].TimeData.sigmoid_ratio_roll,  m_StepData[step_idx].TimeData.sigmoid_distortion_roll);
			b_move = wsigmoid(time, foot_move_period_time, ssp_time_start, b_move_amp, b_move_amp_shift, m_StepData[step_idx].TimeData.sigmoid_ratio_pitch, m_StepData[step_idx].TimeData.sigmoid_distortion_pitch);
			c_move = wsigmoid(time, foot_move_period_time, ssp_time_start, c_move_amp, c_move_amp_shift, m_StepData[step_idx].TimeData.sigmoid_ratio_yaw,   m_StepData[step_idx].TimeData.sigmoid_distortion_yaw);
			z_vibe = wsin(time, foot_move_period_time, z_vibe_phase_shift + 2.0*PI*ssp_time_start/foot_move_period_time, z_vibe_amp, z_vibe_amp_shift);

			if(m_StepData[step_idx].PositionData.bMovingFoot == RFootMove)
			{
				m_matvGtoRF[ep_idx] = GetTransformMatrix(x_move, y_move, z_move + z_vibe, a_move, b_move, c_move);
				m_matvGtoLF[ep_idx] = GetTransformMatrix(m_StepData[step_idx].PositionData.stLeftFootPosition.x, m_StepData[step_idx].PositionData.stLeftFootPosition.y, m_StepData[step_idx].PositionData.stLeftFootPosition.z,
														 m_StepData[step_idx].PositionData.stLeftFootPosition.roll, m_StepData[step_idx].PositionData.stLeftFootPosition.pitch, m_StepData[step_idx].PositionData.stLeftFootPosition.yaw);
				if(time < (ssp_time_start + ssp_time_end)*0.5)
					m_Balancing_Idx(ep_idx) = 2;
				else
					m_Balancing_Idx(ep_idx) = 3;

			}
			else if(m_StepData[step_idx].PositionData.bMovingFoot == LFootMove)	{
				m_matvGtoRF[ep_idx] = GetTransformMatrix(m_StepData[step_idx].PositionData.stRightFootPosition.x, m_StepData[step_idx].PositionData.stRightFootPosition.y, m_StepData[step_idx].PositionData.stRightFootPosition.z,
														 m_StepData[step_idx].PositionData.stRightFootPosition.roll, m_StepData[step_idx].PositionData.stRightFootPosition.pitch, m_StepData[step_idx].PositionData.stRightFootPosition.yaw);
				m_matvGtoLF[ep_idx] = GetTransformMatrix(x_move, y_move, z_move + z_vibe, a_move, b_move, c_move);
				if(time < (ssp_time_start + ssp_time_end)*0.5)
					m_Balancing_Idx(ep_idx) = 6;
				else
					m_Balancing_Idx(ep_idx) = 7;
			}
			else {

				m_matvGtoRF[ep_idx] = GetTransformMatrix(m_StepData[step_idx].PositionData.stRightFootPosition.x, m_StepData[step_idx].PositionData.stRightFootPosition.y, m_StepData[step_idx].PositionData.stRightFootPosition.z,
														 m_StepData[step_idx].PositionData.stRightFootPosition.roll, m_StepData[step_idx].PositionData.stRightFootPosition.pitch, m_StepData[step_idx].PositionData.stRightFootPosition.yaw);
				m_matvGtoLF[ep_idx] = GetTransformMatrix(m_StepData[step_idx].PositionData.stLeftFootPosition.x, m_StepData[step_idx].PositionData.stLeftFootPosition.y, m_StepData[step_idx].PositionData.stLeftFootPosition.z,
														 m_StepData[step_idx].PositionData.stLeftFootPosition.roll, m_StepData[step_idx].PositionData.stLeftFootPosition.pitch, m_StepData[step_idx].PositionData.stLeftFootPosition.yaw);

				m_Balancing_Idx(ep_idx) = 0;
			}
		}

		for(int ep_idx = m_DetailTimeIdx(step_idx*3-1); ep_idx < m_DetailTimeIdx(step_idx*3-0); ep_idx++)
		{
			x_move = wsigmoid(ssp_time_end, foot_move_period_time, ssp_time_start, x_move_amp, x_move_amp_shift, m_StepData[step_idx].TimeData.sigmoid_ratio_x,     m_StepData[step_idx].TimeData.sigmoid_distortion_x);
			y_move = wsigmoid(ssp_time_end, foot_move_period_time, ssp_time_start, y_move_amp, y_move_amp_shift, m_StepData[step_idx].TimeData.sigmoid_ratio_y,     m_StepData[step_idx].TimeData.sigmoid_distortion_y);
			z_move = wsigmoid(ssp_time_end, foot_move_period_time, ssp_time_start, z_move_amp, z_move_amp_shift, m_StepData[step_idx].TimeData.sigmoid_ratio_z,     m_StepData[step_idx].TimeData.sigmoid_distortion_z);
			a_move = wsigmoid(ssp_time_end, foot_move_period_time, ssp_time_start, a_move_amp, a_move_amp_shift, m_StepData[step_idx].TimeData.sigmoid_ratio_roll,  m_StepData[step_idx].TimeData.sigmoid_distortion_roll);
			b_move = wsigmoid(ssp_time_end, foot_move_period_time, ssp_time_start, b_move_amp, b_move_amp_shift, m_StepData[step_idx].TimeData.sigmoid_ratio_pitch, m_StepData[step_idx].TimeData.sigmoid_distortion_pitch);
			c_move = wsigmoid(ssp_time_end, foot_move_period_time, ssp_time_start, c_move_amp, c_move_amp_shift, m_StepData[step_idx].TimeData.sigmoid_ratio_yaw,   m_StepData[step_idx].TimeData.sigmoid_distortion_yaw);
			z_vibe = wsin(ssp_time_end, foot_move_period_time, z_vibe_phase_shift + 2.0*PI*ssp_time_start/foot_move_period_time, z_vibe_amp, z_vibe_amp_shift);

			if(m_StepData[step_idx].PositionData.bMovingFoot == RFootMove)
			{
				m_matvGtoRF[ep_idx] = GetTransformMatrix(x_move, y_move, z_move + z_vibe, a_move, b_move, c_move);
				m_matvGtoLF[ep_idx] = GetTransformMatrix(m_StepData[step_idx].PositionData.stLeftFootPosition.x, m_StepData[step_idx].PositionData.stLeftFootPosition.y, m_StepData[step_idx].PositionData.stLeftFootPosition.z,
														 m_StepData[step_idx].PositionData.stLeftFootPosition.roll, m_StepData[step_idx].PositionData.stLeftFootPosition.pitch, m_StepData[step_idx].PositionData.stLeftFootPosition.yaw);
				m_Balancing_Idx(ep_idx) = 4;
			}
			else if(m_StepData[step_idx].PositionData.bMovingFoot == LFootMove)	{
				m_matvGtoRF[ep_idx] = GetTransformMatrix(m_StepData[step_idx].PositionData.stRightFootPosition.x, m_StepData[step_idx].PositionData.stRightFootPosition.y, m_StepData[step_idx].PositionData.stRightFootPosition.z,
														 m_StepData[step_idx].PositionData.stRightFootPosition.roll, m_StepData[step_idx].PositionData.stRightFootPosition.pitch, m_StepData[step_idx].PositionData.stRightFootPosition.yaw);
				m_matvGtoLF[ep_idx] = GetTransformMatrix(x_move, y_move, z_move + z_vibe, a_move, b_move, c_move);
				m_Balancing_Idx(ep_idx) = 8;
			}
			else {

				m_matvGtoRF[ep_idx] = GetTransformMatrix(m_StepData[step_idx].PositionData.stRightFootPosition.x, m_StepData[step_idx].PositionData.stRightFootPosition.y, m_StepData[step_idx].PositionData.stRightFootPosition.z,
														 m_StepData[step_idx].PositionData.stRightFootPosition.roll, m_StepData[step_idx].PositionData.stRightFootPosition.pitch, m_StepData[step_idx].PositionData.stRightFootPosition.yaw);
				m_matvGtoLF[ep_idx] = GetTransformMatrix(m_StepData[step_idx].PositionData.stLeftFootPosition.x, m_StepData[step_idx].PositionData.stLeftFootPosition.y, m_StepData[step_idx].PositionData.stLeftFootPosition.z,
														 m_StepData[step_idx].PositionData.stLeftFootPosition.roll, m_StepData[step_idx].PositionData.stLeftFootPosition.pitch, m_StepData[step_idx].PositionData.stLeftFootPosition.yaw);

				m_Balancing_Idx(ep_idx) = 0;
			}
		}
	}

	step_idx = m_StepData.size()-1;
	for(int ep_idx = m_DetailTimeIdx(m_DetailTimeIdx.size()-2); ep_idx < m_PatternDataSize; ep_idx++)
	{
		m_matvGtoRF[ep_idx] = GetTransformMatrix(m_StepData[step_idx].PositionData.stRightFootPosition.x, m_StepData[step_idx].PositionData.stRightFootPosition.y, m_StepData[step_idx].PositionData.stRightFootPosition.z,
												 m_StepData[step_idx].PositionData.stRightFootPosition.roll, m_StepData[step_idx].PositionData.stRightFootPosition.pitch, m_StepData[step_idx].PositionData.stRightFootPosition.yaw);
		m_matvGtoLF[ep_idx] = GetTransformMatrix(m_StepData[step_idx].PositionData.stLeftFootPosition.x, m_StepData[step_idx].PositionData.stLeftFootPosition.y, m_StepData[step_idx].PositionData.stLeftFootPosition.z,
												 m_StepData[step_idx].PositionData.stLeftFootPosition.roll, m_StepData[step_idx].PositionData.stLeftFootPosition.pitch, m_StepData[step_idx].PositionData.stLeftFootPosition.yaw);
		m_Balancing_Idx(ep_idx) = 9;
	}
}

void RecursiveWalking::CalcCOBData(matd x_COB, matd y_COB)
{
	double time = 0;
	double ref_time = 0;
	int step_idx = 0;

	double period_time, body_move_period_time;
	double z_swap_amp = 0, z_swap_amp_shift = 0;
	double z_swap_phase_shift = PI*0.5;
	double bz_move_amp = 0.0, bz_move_amp_shift = 0.0;
	double ba_move_amp = 0.0, ba_move_amp_shift = 0.0;
	double bb_move_amp = 0.0, bb_move_amp_shift = 0.0;
	double bc_move_amp = 0.0, bc_move_amp_shift = 0.0;
	double z_swap, bz_move = 0.0, ba_move = 0.0, bb_move = 0.0, bc_move = 0.0;

	double cob_x, cob_y;

	ref_time = 0;
	period_time = m_StepData[step_idx].TimeData.dAbsStepTime - ref_time;
	body_move_period_time = period_time;

	z_swap_amp = m_StepData[step_idx].PositionData.dZ_Swap_Amplitude;
	z_swap_amp_shift = z_swap_amp;

	bz_move_amp = m_StepData[step_idx].PositionData.stBodyPosition.z - m_ReferenceGtoBodyPosition.z;
	bz_move_amp_shift = m_ReferenceGtoBodyPosition.z;

	ba_move_amp = m_StepData[step_idx].PositionData.stBodyPosition.roll - m_ReferenceGtoBodyPosition.roll;
	ba_move_amp_shift = m_ReferenceGtoBodyPosition.roll;

	bb_move_amp = m_StepData[step_idx].PositionData.stBodyPosition.pitch - m_ReferenceGtoBodyPosition.pitch;
	bb_move_amp_shift = m_ReferenceGtoBodyPosition.pitch;

	bc_move_amp = m_StepData[step_idx].PositionData.stBodyPosition.yaw - m_ReferenceGtoBodyPosition.yaw;
	bc_move_amp_shift = m_ReferenceGtoBodyPosition.yaw;

	for(int cob_idx = 0; cob_idx < m_DetailTimeIdx(0); cob_idx++)
	{
		time = cob_idx*TIME_UNIT - ref_time;
		z_swap  = wsin(time, body_move_period_time, z_swap_phase_shift, z_swap_amp, z_swap_amp_shift);
		bz_move = wsigmoid(time, body_move_period_time, 0, bz_move_amp, bz_move_amp_shift, 1.0, 1.0);
		ba_move = wsigmoid(time, body_move_period_time, 0, ba_move_amp, ba_move_amp_shift, 1.0, 1.0);
		bb_move = wsigmoid(time, body_move_period_time, 0, bb_move_amp, bb_move_amp_shift, 1.0, 1.0);
		bc_move = wsigmoid(time, body_move_period_time, 0, bc_move_amp, bc_move_amp_shift, 1.0, 1.0);

		cob_x = x_COB(cob_idx, 0); cob_y = y_COB(cob_idx, 0);
		m_matvGtoCOB[cob_idx] = GetTransformMatrix(cob_x, cob_y, bz_move+z_swap, ba_move, bb_move, bc_move);
	}

	for(step_idx = 1; step_idx < m_StepData.size() - 1; step_idx++)
	{
		ref_time = m_StepData[step_idx-1].TimeData.dAbsStepTime;
		period_time = m_StepData[step_idx].TimeData.dAbsStepTime - ref_time;
		body_move_period_time = period_time;

		z_swap_amp = m_StepData[step_idx].PositionData.dZ_Swap_Amplitude;
		z_swap_amp_shift = z_swap_amp;

		bz_move_amp = m_StepData[step_idx].PositionData.stBodyPosition.z - m_StepData[step_idx-1].PositionData.stBodyPosition.z;
		bz_move_amp_shift = m_StepData[step_idx-1].PositionData.stBodyPosition.z;

		ba_move_amp = m_StepData[step_idx].PositionData.stBodyPosition.roll - m_StepData[step_idx-1].PositionData.stBodyPosition.roll;
		ba_move_amp_shift = m_StepData[step_idx-1].PositionData.stBodyPosition.roll;

		bb_move_amp = m_StepData[step_idx].PositionData.stBodyPosition.pitch - m_StepData[step_idx-1].PositionData.stBodyPosition.pitch;
		bb_move_amp_shift = m_StepData[step_idx-1].PositionData.stBodyPosition.pitch;

		bc_move_amp = m_StepData[step_idx].PositionData.stBodyPosition.yaw - m_StepData[step_idx-1].PositionData.stBodyPosition.yaw;
		bc_move_amp_shift = m_StepData[step_idx-1].PositionData.stBodyPosition.yaw;



		//refer to detail_time_idx
		for(int cob_idx = m_DetailTimeIdx(step_idx*3-3); cob_idx < m_DetailTimeIdx(step_idx*3-0); cob_idx++)
		{
			time = cob_idx*TIME_UNIT - ref_time;
			z_swap  = wsin(time, body_move_period_time, z_swap_phase_shift, z_swap_amp, z_swap_amp_shift);
			bz_move = wsigmoid(time, body_move_period_time, 0, bz_move_amp, bz_move_amp_shift, 1.0, 1.0);
			ba_move = wsigmoid(time, body_move_period_time, 0, ba_move_amp, ba_move_amp_shift, 1.0, 1.0);
			bb_move = wsigmoid(time, body_move_period_time, 0, bb_move_amp, bb_move_amp_shift, 1.0, 1.0);
			bc_move = wsigmoid(time, body_move_period_time, 0, bc_move_amp, bc_move_amp_shift, 1.0, 1.0);

			cob_x = x_COB(cob_idx, 0); cob_y = y_COB(cob_idx, 0);
			m_matvGtoCOB[cob_idx] = GetTransformMatrix(cob_x, cob_y, bz_move+z_swap, ba_move, bb_move, bc_move);
		}
	}

	step_idx = m_StepData.size()-1;

	ref_time = m_StepData[step_idx-1].TimeData.dAbsStepTime;
	period_time = m_StepData[step_idx].TimeData.dAbsStepTime - ref_time;
	body_move_period_time = period_time;

	z_swap_amp = m_StepData[step_idx].PositionData.dZ_Swap_Amplitude;
	z_swap_amp_shift = z_swap_amp;

	bz_move_amp = m_StepData[step_idx].PositionData.stBodyPosition.z - m_StepData[step_idx-1].PositionData.stBodyPosition.z;
	bz_move_amp_shift = m_StepData[step_idx-1].PositionData.stBodyPosition.z;

	ba_move_amp = m_StepData[step_idx].PositionData.stBodyPosition.roll - m_StepData[step_idx-1].PositionData.stBodyPosition.roll;
	ba_move_amp_shift = m_StepData[step_idx-1].PositionData.stBodyPosition.roll;

	bb_move_amp = m_StepData[step_idx].PositionData.stBodyPosition.pitch - m_StepData[step_idx-1].PositionData.stBodyPosition.pitch;
	bb_move_amp_shift = m_StepData[step_idx-1].PositionData.stBodyPosition.pitch;

	bc_move_amp = m_StepData[step_idx].PositionData.stBodyPosition.yaw - m_StepData[step_idx-1].PositionData.stBodyPosition.yaw;
	bc_move_amp_shift = m_StepData[step_idx-1].PositionData.stBodyPosition.yaw;

	for(int cob_idx = m_DetailTimeIdx(m_DetailTimeIdx.size()-2); cob_idx < m_PatternDataSize; cob_idx++)
	{
		time = cob_idx*TIME_UNIT - ref_time;
		z_swap  = wsin(time, body_move_period_time, z_swap_phase_shift, z_swap_amp, z_swap_amp_shift);
		bz_move = wsigmoid(time, body_move_period_time, 0, bz_move_amp, bz_move_amp_shift, 1.0, 1.0);
		ba_move = wsigmoid(time, body_move_period_time, 0, ba_move_amp, ba_move_amp_shift, 1.0, 1.0);
		bb_move = wsigmoid(time, body_move_period_time, 0, bb_move_amp, bb_move_amp_shift, 1.0, 1.0);
		bc_move = wsigmoid(time, body_move_period_time, 0, bc_move_amp, bc_move_amp_shift, 1.0, 1.0);

		cob_x = x_COB(cob_idx, 0); cob_y = y_COB(cob_idx, 0);
		m_matvGtoCOB[cob_idx] = GetTransformMatrix(cob_x, cob_y, bz_move+z_swap, ba_move, bb_move, bc_move);
	}
}

bool RecursiveWalking::CalcZMPbyMPMM()
{
	Pose3D epr, epl;
	double angle[12] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
	double rangle[6] = { 0, 0, 0, 0, 0, 0}, langle[6] = { 0, 0, 0, 0, 0, 0};

	matd v1, v2, a;
	double sum1 = 0.0, sum2 = 1.0;

	//0
	matCOBtoG = GetTransformMatrixInverse(m_matvGtoCOB[0]);
	matRHtoRF = matRHtoCOB*matCOBtoG*m_matvGtoRF[0];
	matLHtoLF = matLHtoCOB*matCOBtoG*m_matvGtoLF[0];

	epr = GetPose3DfromTransformMatrix(matRHtoRF);
	epl = GetPose3DfromTransformMatrix(matLHtoLF);

	if(computeIK(&angle[0], epr.x, epr.y, epr.z + Kinematics::LEG_LENGTH, epr.roll, epr.pitch, epr.yaw) == false)
	{
		return false;
		printf("IK not Solved\n");
	}


	if(computeIK(&angle[6], epl.x, epl.y, epl.z + Kinematics::LEG_LENGTH, epl.roll, epl.pitch, epl.yaw) == false)
	{
		printf("IK not Solved\n");
		return false;
	}

	int step_idx = m_StepIdxData(0);
	r_arm[0] = (double)dir[12]*(epr.x - epl.x)*m_StepData[step_idx].PositionData.dShoulderSwingGain + r_arm_init[0];
	r_arm[3] = (double)dir[14]*(epr.x - epl.x)*m_StepData[step_idx].PositionData.dElbowSwingGain    + r_arm_init[3];

	l_arm[0] = (double)dir[13]*(epl.x - epr.x)*m_StepData[step_idx].PositionData.dShoulderSwingGain + l_arm_init[0];
	l_arm[3] = (double)dir[15]*(epl.x - epr.x)*m_StepData[step_idx].PositionData.dElbowSwingGain    + l_arm_init[3];

	for(int idx = 0; idx < 6; idx++)
	{
		rangle[idx] = angle[idx]*dir[idx]*180.0/PI;
		langle[idx] = angle[idx+6]*dir[idx+6]*180.0/PI;
	}

	PData1 = m_matvGtoCOB[0]*GetCOMofMultiBody(langle, rangle, l_arm, r_arm);


	//1
	matCOBtoG = GetTransformMatrixInverse(m_matvGtoCOB[1]);
	matRHtoRF = matRHtoCOB*matCOBtoG*m_matvGtoRF[1];
	matLHtoLF = matLHtoCOB*matCOBtoG*m_matvGtoLF[1];

	epr = GetPose3DfromTransformMatrix(matRHtoRF);
	epl = GetPose3DfromTransformMatrix(matLHtoLF);

	if(computeIK(&angle[0], epr.x, epr.y, epr.z + Kinematics::LEG_LENGTH, epr.roll, epr.pitch, epr.yaw) == false)
	{
		return false;
		printf("IK not Solved\n");
	}


	if(computeIK(&angle[6], epl.x, epl.y, epl.z + Kinematics::LEG_LENGTH, epl.roll, epl.pitch, epl.yaw) == false)
	{
		printf("IK not Solved\n");
		return false;
	}

	step_idx = m_StepIdxData(1);
	r_arm[0] = (double)dir[12]*(epr.x - epl.x)*m_StepData[step_idx].PositionData.dShoulderSwingGain + r_arm_init[0];
	r_arm[3] = (double)dir[14]*(epr.x - epl.x)*m_StepData[step_idx].PositionData.dElbowSwingGain    + r_arm_init[3];

	l_arm[0] = (double)dir[13]*(epl.x - epr.x)*m_StepData[step_idx].PositionData.dShoulderSwingGain + l_arm_init[0];
	l_arm[3] = (double)dir[15]*(epl.x - epr.x)*m_StepData[step_idx].PositionData.dElbowSwingGain    + l_arm_init[3];

	for(int idx = 0; idx < 6; idx++)
	{
		rangle[idx] = angle[idx]*dir[idx]*180.0/PI;
		langle[idx] = angle[idx+6]*dir[idx+6]*180.0/PI;
	}

	PData2 = m_matvGtoCOB[1]*GetCOMofMultiBody(langle, rangle, l_arm, r_arm);



	for(int zmp_idx = 2 ; zmp_idx < m_PatternDataSize; zmp_idx++)
	{
		matCOBtoG = GetTransformMatrixInverse(m_matvGtoCOB[zmp_idx]);
		matRHtoRF = matRHtoCOB*matCOBtoG*m_matvGtoRF[zmp_idx];
		matLHtoLF = matLHtoCOB*matCOBtoG*m_matvGtoLF[zmp_idx];

		epr = GetPose3DfromTransformMatrix(matRHtoRF);
		epl = GetPose3DfromTransformMatrix(matLHtoLF);

		if(computeIK(&angle[0], epr.x, epr.y, epr.z + Kinematics::LEG_LENGTH, epr.roll, epr.pitch, epr.yaw) == false)
		{
			printf("IK not Solved. StepNum : %d\n", m_StepIdxData(zmp_idx));
			return false;
		}


		if(computeIK(&angle[6], epl.x, epl.y, epl.z + Kinematics::LEG_LENGTH, epl.roll, epl.pitch, epl.yaw) == false)
		{
			printf("IK not Solved. StepNum : %d\n", m_StepIdxData(zmp_idx));
			return false;
		}

		step_idx = m_StepIdxData(zmp_idx);
		r_arm[0] = (double)dir[12]*(epr.x - epl.x)*m_StepData[step_idx].PositionData.dShoulderSwingGain + r_arm_init[0];
		r_arm[3] = (double)dir[14]*(epr.x - epl.x)*m_StepData[step_idx].PositionData.dElbowSwingGain    + r_arm_init[3];

		l_arm[0] = (double)dir[13]*(epl.x - epr.x)*m_StepData[step_idx].PositionData.dShoulderSwingGain + l_arm_init[0];
		l_arm[3] = (double)dir[15]*(epl.x - epr.x)*m_StepData[step_idx].PositionData.dElbowSwingGain    + l_arm_init[3];

		for(int idx = 0; idx < 6; idx++)
		{
			rangle[idx] = angle[idx]*dir[idx]*180.0/PI;
			langle[idx] = angle[idx+6]*dir[idx+6]*180.0/PI;
		}

		PData3 = m_matvGtoCOB[zmp_idx]*GetCOMofMultiBody(langle, rangle, l_arm, r_arm);

		v1 = (PData2 - PData1)/(TIME_UNIT*0.001);
		v2 = (PData3 - PData2)/(TIME_UNIT*0.001);

		a = (v2 - v1)/(TIME_UNIT*0.001);

		sum1 = 0;  sum2 = 0;
		for(int idx = 0; idx < 22; idx++)
		{
			sum1 += (Dynamics::Mass[idx])*(PData2(0, idx)*(a(2, idx) + G) - a(0, idx)*PData2(2, idx));
			sum2 += (Dynamics::Mass[idx])*(a(2, idx) + G);
			//sum2 += Dynamics::Mass[idx]*(PData2_calc(1, idx)*(a(2, idx) + G) - a(1, idx)*PData2_calc(2, idx));
		}
		m_ShoulderSwingGain[zmp_idx] = m_StepData[step_idx].PositionData.dShoulderSwingGain;
		m_ElbowSwingGain[zmp_idx] = m_StepData[step_idx].PositionData.dElbowSwingGain;
		m_ZMP_Calculated_by_MPMM_X(zmp_idx-1, 0) = sum1/sum2;

		sum1 = 0;
		for(int idx = 0; idx < 22; idx++)
		{
			sum1 += Dynamics::Mass[idx]*(PData2(1, idx)*(a(2, idx) + G) - a(1, idx)*PData2(2, idx));
		}

		m_ZMP_Calculated_by_MPMM_Y(zmp_idx-1, 0) = sum1/sum2;

		PData1 = PData2;
		PData2 = PData3;
	}

	m_ShoulderSwingGain(0) = m_ShoulderSwingGain(1) = m_ShoulderSwingGain(2);
	m_ElbowSwingGain(0) = m_ElbowSwingGain(1) = m_ElbowSwingGain(2);
	m_ZMP_Calculated_by_MPMM_X(0,0) = m_ZMP_Calculated_by_MPMM_X(1,0);
	m_ZMP_Calculated_by_MPMM_Y(0,0) = m_ZMP_Calculated_by_MPMM_Y(1,0);

	m_ZMP_Calculated_by_MPMM_X(m_PatternDataSize-1,0) = m_ZMP_Calculated_by_MPMM_X(m_PatternDataSize-2,0);
	m_ZMP_Calculated_by_MPMM_Y(m_PatternDataSize-1,0) = m_ZMP_Calculated_by_MPMM_Y(m_PatternDataSize-2,0);


	return true;
}

bool RecursiveWalking::CalcWalkingPattern()
{
	m_IsCompleteCalcPattern = false;
	m_Real_Running = false;

	printf("Calc Walking Pattern Start\n");

	if(m_StepData.size() < 3) {
		printf("there is no step data\n");
		return false;
	}

	int StepDataSize = m_StepData.size();
	m_PatternDataSize = (m_StepData[StepDataSize-1].TimeData.dAbsStepTime)/(double)TIME_UNIT;

	m_DetailTimeIdx.resize((StepDataSize - 2)*3+2);
	m_StepIdxData.resize(m_PatternDataSize);
	m_Balancing_Idx.resize(m_PatternDataSize);
	A.resize(m_PatternDataSize, m_PatternDataSize);
	IA.resize(m_PatternDataSize, m_PatternDataSize);

	x_LIPM.resize(m_PatternDataSize, 1);
	y_LIPM.resize(m_PatternDataSize, 1);

	x_MPMM.resize(m_PatternDataSize, 1);
	y_MPMM.resize(m_PatternDataSize, 1);
//	x_MPMM_pre.resize(m_PatternDataSize, 1);
//	y_MPMM_pre.resize(m_PatternDataSize, 1);
//	x_MPMM_post.resize(m_PatternDataSize, 1);
//	y_MPMM_post.resize(m_PatternDataSize, 1);
	x_delta.resize(m_PatternDataSize, 1);
	y_delta.resize(m_PatternDataSize, 1);

	m_ZMP_Reference_X.resize(m_PatternDataSize, 1);
	m_ZMP_Reference_Y.resize(m_PatternDataSize, 1);
	m_ZMP_Calculated_by_MPMM_X.resize(m_PatternDataSize, 1);
	m_ZMP_Calculated_by_MPMM_Y.resize(m_PatternDataSize, 1);

//	m_ZMP_Calculated_by_MPMM_X_pre.resize(m_PatternDataSize, 1);
//	m_ZMP_Calculated_by_MPMM_Y_pre.resize(m_PatternDataSize, 1);
//	m_ZMP_Calculated_by_MPMM_X_post.resize(m_PatternDataSize, 1);
//	m_ZMP_Calculated_by_MPMM_Y_post.resize(m_PatternDataSize, 1);

	m_matvGtoRF.resize(m_PatternDataSize);
	m_matvGtoLF.resize(m_PatternDataSize);
	m_matvGtoCOB.resize(m_PatternDataSize);

	m_ElbowSwingGain.resize(m_PatternDataSize);
	m_ShoulderSwingGain.resize(m_PatternDataSize);
	//printf("process1\n");
	//A Initialize
	double a, b, c;
	a = -(m_PresentGtoBodyPosition.z - 0.5*(m_PresentGtoLeftFootPosition.z + m_PresentGtoRightFootPosition.z))/(G*TIME_UNIT*0.001*TIME_UNIT*0.001);
	b = -2*a + 1;
	c = a;

	A.fill(0);
	A(0, 0) = a+b;
	A(0, 1) = c;
	for(int row = 1; row < m_PatternDataSize-1; row++)
	{
		A(row, row-1) = a;
		A(row, row)   = b;
		A(row, row+1) = c;
	}
	A(m_PatternDataSize-1, m_PatternDataSize-2) = a;
	A(m_PatternDataSize-1, m_PatternDataSize-1) = b+c;

	IA = A.inverse();
	//printf("process2\n");

	CalcIdxData();
	//printf("process3\n");
	CalcDetailTimeIdx();
	//printf("process4\n");
	CalcRefZMP();
	//printf("process5\n");

	x_LIPM = IA*m_ZMP_Reference_X;
	y_LIPM = IA*m_ZMP_Reference_Y;

	CalcEndPointData();
	//printf("process6\n");
	CalcCOBData(x_LIPM, y_LIPM);
	//printf("process7\n");
	if(CalcZMPbyMPMM() == false)
	{
		printf("please modify step data\n");
		return false;
	}

	x_delta = IA*(m_ZMP_Reference_X-m_ZMP_Calculated_by_MPMM_X);
	y_delta = IA*(m_ZMP_Reference_Y-m_ZMP_Calculated_by_MPMM_Y);

	x_MPMM = x_LIPM + x_delta;
	y_MPMM = y_LIPM + y_delta;

	for(int calc_num = 0 ; calc_num < 10; calc_num++)
	{
		CalcCOBData(x_MPMM, y_MPMM);
		if(CalcZMPbyMPMM() == false)
		{
			printf("please modify step data calc num :%d\n", calc_num);
			return false;
		}

		x_delta = IA*(m_ZMP_Reference_X-m_ZMP_Calculated_by_MPMM_X);
		y_delta = IA*(m_ZMP_Reference_Y-m_ZMP_Calculated_by_MPMM_Y);

		x_MPMM = x_MPMM + x_delta;
		y_MPMM = y_MPMM + y_delta;
	}

	CalcCOBData(x_MPMM, y_MPMM);
	if(CalcZMPbyMPMM() == false)
	{
		printf("please modify step data\n");
		return false;
	}


	printf("Pattern Calc Complete\n");
	m_StepData.clear();

	printf("Wait End of play\n");
	while(m_Real_Running);

	m_matvGtoRFforPlaying = m_matvGtoRF;
	m_matvGtoLFforPlaying = m_matvGtoLF;
	m_matvGtoCOBforPlaying = m_matvGtoCOB;
	m_ElbowSwingGainforPlaying = m_ElbowSwingGain;
	m_ShoulderSwingGainforPlaying = m_ShoulderSwingGain;
	m_IsCompleteCalcPattern = true;
	printf("All Process Complete\n");

//	for(int i =0; i < m_PatternDataSize; i++)
//	{
//		//printf("%f %f %f %f %f %f\n", m_ZMP_Reference_X(i), x_LIPM(i), x_MPMM(i), m_ZMP_Calculated_by_MPMM_X(i), m_matvGtoRFforPlaying[i](0,3), m_matvGtoLFforPlaying[i](0,3));
//		printf("%f %f %f %f %f %f\n", m_ZMP_Reference_Y(i), y_LIPM(i), y_MPMM(i), m_ZMP_Calculated_by_MPMM_Y(i), m_matvGtoRFforPlaying[i](1,3), m_matvGtoLFforPlaying[i](1,3));
//	}

	return true;
}

void RecursiveWalking::CalcDHforRightArm(double *rightArmAngle_deg, matd *T01, matd *T02, matd *T03, matd *T04, matd *T05, matd *T06)
{
	matd t1, t2, t3, t4, t5, t6;
	t1 = CalcDH(     0,  PI/2.0,     0, rightArmAngle_deg[0]*PI/180.0);
	t2 = CalcDH(     0, -PI/2.0,     0, rightArmAngle_deg[1]*PI/180.0);
	t3 = CalcDH(  30.0, -PI/2.0, 246.0, rightArmAngle_deg[2]*PI/180.0);
	t4 = CalcDH( -30.0,  PI/2.0,     0, rightArmAngle_deg[3]*PI/180.0);
	//t5 = CalcDH(     0, -PI/2.0, 135.0, rightArmAngle_deg[4]*PI/180.0);
	t5 = CalcDH(     0, -PI/2.0, 304.0, rightArmAngle_deg[4]*PI/180.0);
	t6 = CalcDH(     0,       0,     0, rightArmAngle_deg[5]*PI/180.0);

	*T01 = t1;
	*T02 = (*T01)*t2;
	*T03 = (*T02)*t3;
	*T04 = (*T03)*t4;
	*T05 = (*T04)*t5;
	*T06 = (*T05)*t6;
}

void RecursiveWalking::CalcDHforLeftArm(double *leftArmAngle_deg, matd *T01, matd *T02, matd *T03, matd *T04, matd *T05, matd *T06)
{
	matd t1, t2, t3, t4, t5, t6;
	t1 = CalcDH(     0, -PI/2.0,     0, leftArmAngle_deg[0]*PI/180.0);
	t2 = CalcDH(     0,  PI/2.0,     0, leftArmAngle_deg[1]*PI/180.0);
	t3 = CalcDH(  30.0,  PI/2.0, 246.0, leftArmAngle_deg[2]*PI/180.0);
	t4 = CalcDH( -30.0, -PI/2.0,     0, leftArmAngle_deg[3]*PI/180.0);
	//t5 = CalcDH(     0,  PI/2.0, 135.0, leftArmAngle_deg[4]*PI/180.0);
	t5 = CalcDH(     0,  PI/2.0, 304.0, leftArmAngle_deg[4]*PI/180.0);
	t6 = CalcDH(     0,       0,     0, leftArmAngle_deg[5]*PI/180.0);

	*T01 = t1;
	*T02 = (*T01)*t2;
	*T03 = (*T02)*t3;
	*T04 = (*T03)*t4;
	*T05 = (*T04)*t5;
	*T06 = (*T05)*t6;
}

void RecursiveWalking::CalcDHforRightLeg(double *rightLegAngle_deg, matd *T01, matd *T02, matd *T03, matd *T04, matd *T05, matd *T06)
{
	double th[6];
	static const double l1 = Kinematics::THIGH_LENGTH, l2 = Kinematics::CALF_LENGTH, l3 = Kinematics::ANKLE_LENGTH;
	matd t1, t2, t3, t4, t5, t6;

	th[0] = ( -rightLegAngle_deg[0] + 90.0  )*PI/180.0;
	th[1] = (  rightLegAngle_deg[1] + 90.0  )*PI/180.0;
	th[2] = (  rightLegAngle_deg[2]         )*PI/180.0;
	th[3] = (  rightLegAngle_deg[3]         )*PI/180.0;
	th[4] = (  rightLegAngle_deg[4]         )*PI/180.0;
	th[5] = (  rightLegAngle_deg[5]         )*PI/180.0;

	t1 = CalcDH( 0, -PI/2.0, 0, th[0] );
	t2 = CalcDH( 0,  PI/2.0, 0, th[1] );
	t3 = CalcDH(l1,       0, 0, th[2] );
	t4 = CalcDH(l2,      PI, 0, th[3] );
	t5 = CalcDH( 0, -PI/2.0, 0, th[4] );
	t6 = CalcDH(l3,       0, 0, th[5] );

	*T01 = t1;
	*T02 = (*T01)*t2;
	*T03 = (*T02)*t3;
	*T04 = (*T03)*t4;
	*T05 = (*T04)*t5;
	*T06 = (*T05)*t6;
}

void RecursiveWalking::CalcDHforLeftLeg(double *leftLegAngle_deg, matd *T01, matd *T02, matd *T03, matd *T04, matd *T05, matd *T06)
{
	double th[6];
	//static const double l1 = 301.49627, l2 = 301.49627, l3 = 118.0;
	static const double l1 = Kinematics::THIGH_LENGTH, l2 = Kinematics::CALF_LENGTH, l3 = Kinematics::ANKLE_LENGTH;
	matd t1, t2, t3, t4, t5, t6;

	th[0] = ( -leftLegAngle_deg[0] + 90.0  )*PI/180.0;
	th[1] = (  leftLegAngle_deg[1] + 90.0  )*PI/180.0;
	th[2] = (  leftLegAngle_deg[2]         )*PI/180.0;
	th[3] = (  leftLegAngle_deg[3]         )*PI/180.0;
	th[4] = (  leftLegAngle_deg[4]         )*PI/180.0;
	th[5] = (  leftLegAngle_deg[5]         )*PI/180.0;

	t1 = CalcDH( 0, -PI/2.0, 0, th[0] );
	t2 = CalcDH( 0, -PI/2.0, 0, th[1] );
	t3 = CalcDH(l1,       0, 0, th[2] );
	t4 = CalcDH(l2,      PI, 0, th[3] );
	t5 = CalcDH( 0,  PI/2.0, 0, th[4] );
	t6 = CalcDH(l3,       0, 0, th[5] );

	*T01 = t1;
	*T02 = (*T01)*t2;
	*T03 = (*T02)*t3;
	*T04 = (*T03)*t4;
	*T05 = (*T04)*t5;
	*T06 = (*T05)*t6;
}

matd RecursiveWalking::GetCOMofMultiBody(double* leftLegAngle_deg, double* rightLegAngle_deg, double* leftArmAngle_deg, double* rightArmAngle_deg)
{
	matd T01, T02, T03, T04, T05, T06;
	matd COM_Pos_List(4, 22);
	static vecd vCOM(4); vCOM.fill(1);


	//0~5 RightLeg
	CalcDHforRightLeg(rightLegAngle_deg, &T01, &T02, &T03, &T04, &T05, &T06);
	vCOM(0) = 0; vCOM(1) = -18.8; vCOM(2) = 47.8;
	COM_Pos_List.col(0) = matCOBtoRH*T01*vCOM;

	vCOM(0) = 0; vCOM(1) = 21.6; vCOM(2) = 0;
	COM_Pos_List.col(1) = matCOBtoRH*T02*vCOM;

	vCOM(0) = -129.22547; vCOM(1) = -13.9106; vCOM(2) = -29.4;
	COM_Pos_List.col(2) = matCOBtoRH*T03*vCOM;

	vCOM(0) = -119.0632; vCOM(1) = -19.3734; vCOM(2) = 31.3;
	COM_Pos_List.col(3) = matCOBtoRH*T04*vCOM;

	vCOM(0) = 0; vCOM(1) = 0; vCOM(2) = -11.2;
	COM_Pos_List.col(4) = matCOBtoRH*T05*vCOM;

	vCOM(0) = -31.49254; vCOM(1) = 0; vCOM(2) = -5.3;
	COM_Pos_List.col(5) = matCOBtoRH*T06*vCOM;

	//6~11 LeftLeg
	CalcDHforLeftLeg(leftLegAngle_deg, &T01, &T02, &T03, &T04, &T05, &T06);
	vCOM(0) = 0; vCOM(1) = -18.8; vCOM(2) = 47.8;
	COM_Pos_List.col(6) = matCOBtoLH*T01*vCOM;

	vCOM(0) = 0; vCOM(1) = -21.6; vCOM(2) = 0;
	COM_Pos_List.col(7) = matCOBtoLH*T02*vCOM;

	vCOM(0) = -129.22547; vCOM(1) = 13.9106; vCOM(2) = -29.4;
	COM_Pos_List.col(8) = matCOBtoLH*T03*vCOM;

	vCOM(0) = -119.0632; vCOM(1) = 19.3734; vCOM(2) = 31.3;
	COM_Pos_List.col(9) = matCOBtoLH*T04*vCOM;

	vCOM(0) = 0; vCOM(1) = 0; vCOM(2) = -11.2;
	COM_Pos_List.col(10) = matCOBtoLH*T05*vCOM;

	vCOM(0) = -31.49254; vCOM(1) = 0; vCOM(2) = -5.3;
	COM_Pos_List.col(11) = matCOBtoLH*T06*vCOM;

	//12~15 RightArm
	matd matCOBtoRSholder = GetTranslationMatrix(0, -234, 447)*GetOrientationMatrix(PI*0.5, PI*0.5, 0.0);
	CalcDHforRightArm(rightArmAngle_deg, &T01, &T02, &T03, &T04, &T05, &T06);
	vCOM(0) = -22.0; vCOM(1) = 143.3; vCOM(2) = 0.0;
	COM_Pos_List.col(12) = matCOBtoRSholder*T03*vCOM;

	vCOM(0) =  0.0; vCOM(1) = 0.0; vCOM(2) = 35.1;
	COM_Pos_List.col(13) = matCOBtoRSholder*T04*vCOM;

	vCOM(0) =  0.0; vCOM(1) = -46.4; vCOM(2) = 0.0;
	COM_Pos_List.col(14) = matCOBtoRSholder*T05*vCOM;
//default
	vCOM(0) = 3.9; vCOM(1) = 146.0; vCOM(2) = 0.0;
	COM_Pos_List.col(15) = matCOBtoRSholder*T06*vCOM;

//	//for DRC
//	vCOM(0) = 0.0; vCOM(1) = 52.0; vCOM(2) = 0.0;
//	COM_Pos_List.col(15) = matCOBtoRSholder*T06*vCOM;


	//16~19 LeftArm
	matd matCOBtoLShoulder = GetTranslationMatrix(0, 234, 447)*GetOrientationMatrix(-PI*0.5, PI*0.5, 0.0);
	CalcDHforLeftArm(leftArmAngle_deg, &T01, &T02, &T03, &T04, &T05, &T06);
	vCOM(0) = -22.0; vCOM(1) = -143.3; vCOM(2) = 0.0;
	COM_Pos_List.col(16) = matCOBtoLShoulder*T03*vCOM;

	vCOM(0) =  0.0; vCOM(1) = 0.0; vCOM(2) = 35.1;
	COM_Pos_List.col(17) = matCOBtoLShoulder*T04*vCOM;

	vCOM(0) =  0.0; vCOM(1) = -46.4; vCOM(2) = 0.0;
	COM_Pos_List.col(18) = matCOBtoLShoulder*T05*vCOM;
	//default
	vCOM(0) = 3.9; vCOM(1) = 146.0; vCOM(2) = 0.0;
	COM_Pos_List.col(19) = matCOBtoLShoulder*T06*vCOM;

//	//for DRC
//	vCOM(0) = 0.0; vCOM(1) = 52.0; vCOM(2) = 0.0;
//	COM_Pos_List.col(19) = matCOBtoRSholder*T06*vCOM;


	//20 body_lower
	vCOM(0) = -26.4; vCOM(1) = 0; vCOM(2) = 161.2;
	COM_Pos_List.col(20) = vCOM;

	//21 body_upper
	vCOM(0) = -20.8; vCOM(1) = 0; vCOM(2) = 155.7;
	COM_Pos_List.col(21) = GetTransformMatrix(0, 0, 282, 0, 0, 0)*vCOM;

	return COM_Pos_List;
}

void RecursiveWalking::Process()
{
	if(m_Ctrl_Running == false)
		return;

	static double angle[16] = {InitAngle[0], InitAngle[1], InitAngle[2], InitAngle[3], InitAngle[4], InitAngle[5],
			 InitAngle[6], InitAngle[7], InitAngle[8], InitAngle[9], InitAngle[10], InitAngle[11],
			 InitAngle[12], InitAngle[13],
			 InitAngle[14], InitAngle[15],
	};
	static matd matGtoCOBp = GetTransformMatrix(m_ReferenceGtoBodyPosition.x, m_ReferenceGtoBodyPosition.y, m_ReferenceGtoBodyPosition.z, m_ReferenceGtoBodyPosition.roll, m_ReferenceGtoBodyPosition.pitch, m_ReferenceGtoBodyPosition.yaw);
	static matd matCOBtoGp = GetTransformMatrixInverse(matGtoCOBp);
	static matd matGtoRFp = GetTransformMatrix(m_ReferenceGtoRightFootPosition.x, m_ReferenceGtoRightFootPosition.y, m_ReferenceGtoRightFootPosition.z, m_ReferenceGtoRightFootPosition.roll, m_ReferenceGtoRightFootPosition.pitch, m_ReferenceGtoRightFootPosition.yaw);
	static matd matGtoLFp = GetTransformMatrix(m_ReferenceGtoLeftFootPosition.x, m_ReferenceGtoLeftFootPosition.y, m_ReferenceGtoLeftFootPosition.z, m_ReferenceGtoLeftFootPosition.roll, m_ReferenceGtoLeftFootPosition.pitch, m_ReferenceGtoLeftFootPosition.yaw);
	static matd matRHtoRFp = matRHtoCOB*matCOBtoGp*matGtoRFp;
	static matd matLHtoLFp = matLHtoCOB*matCOBtoGp*matGtoLFp;

	double ankleLength = Kinematics::ANKLE_LENGTH;
	static int m_Balancing_Index = 0;

	static double elbow_swing_gain = 0.0;
	static double shoulder_swing_gain = 0.0;

	static Pose3D epr, epl;
	if(m_Real_Running == true)
	{
		matGtoCOBp = m_matvGtoCOBforPlaying[m_play_idx];
		matGtoRFp = m_matvGtoRFforPlaying[m_play_idx];
		matGtoLFp = m_matvGtoLFforPlaying[m_play_idx];

		matCOBtoGp = GetTransformMatrixInverse(matGtoCOBp);

		matRHtoRFp = matRHtoCOB*matCOBtoGp*matGtoRFp;
		matLHtoLFp = matLHtoCOB*matCOBtoGp*matGtoLFp;

		shoulder_swing_gain = m_ShoulderSwingGainforPlaying(m_play_idx);
		elbow_swing_gain = m_ShoulderSwingGainforPlaying(m_play_idx);
		//fprintf(fp, "cob_y : %f\n", matGtoCOBp(1,3));
		m_Balancing_Idx(m_play_idx);
		m_dplay_idx += 1.0;

		//m_play_idx += 1;
	}
//	else
//	{
//		matGtoCOBp = GetTransformMatrix(m_ReferenceGtoBodyPosition.x, m_ReferenceGtoBodyPosition.y, m_ReferenceGtoBodyPosition.z, m_ReferenceGtoBodyPosition.roll, m_ReferenceGtoBodyPosition.pitch, m_ReferenceGtoBodyPosition.yaw);
//		matCOBtoGp = GetTransformMatrixInverse(matGtoCOBp);
//		matGtoRFp = GetTransformMatrix(m_ReferenceGtoRightFootPosition.x, m_ReferenceGtoRightFootPosition.y, m_ReferenceGtoRightFootPosition.z, m_ReferenceGtoRightFootPosition.roll, m_ReferenceGtoRightFootPosition.pitch, m_ReferenceGtoRightFootPosition.yaw);
//		matGtoLFp = GetTransformMatrix(m_ReferenceGtoLeftFootPosition.x, m_ReferenceGtoLeftFootPosition.y, m_ReferenceGtoLeftFootPosition.z, m_ReferenceGtoLeftFootPosition.roll, m_ReferenceGtoLeftFootPosition.pitch, m_ReferenceGtoLeftFootPosition.yaw);
//		matRHtoRFp = matRHtoCOB*matCOBtoGp*matGtoRFp;
//		matLHtoLFp = matLHtoCOB*matCOBtoGp*matGtoLFp;
//		epr = GetPose3DfromTransformMatrix(matRHtoRFp);
//		epl = GetPose3DfromTransformMatrix(matLHtoLFp);
//	}


	epr = GetPose3DfromTransformMatrix(matRHtoRFp);
	epl = GetPose3DfromTransformMatrix(matLHtoLFp);

#ifdef REAL_ROBOT
	//Balancing Algorithm
	if(BALANCE_ENABLE)
	{
		static bool bIslanded = true;
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

		for(int i = 1 ; i < FT_SENSOR_WINDOW_SIZE ; i++)
		{
			m_right_leg_ft_fz_N_array[i-1] = m_right_leg_ft_fz_N_array[i];
			m_left_leg_ft_fz_N_array[i-1] = m_left_leg_ft_fz_N_array[i];
		}
		m_right_leg_ft_fz_N_array[FT_SENSOR_WINDOW_SIZE - 1] = right_leg_fz_N;
		m_left_leg_ft_fz_N_array[FT_SENSOR_WINDOW_SIZE - 1] = left_leg_fz_N;


		double right_zmpx_sensed_by_ft_mm  =  (1000.0*right_leg_Ty_Nm + ankleLength*right_leg_fx_N)/right_leg_fz_N;
		double right_zmpy_sensed_by_ft_mm  = -(-1000.0*right_leg_Tx_Nm + ankleLength*right_leg_fy_N)/right_leg_fz_N;

		double left_zmpx_sensed_by_ft_mm  =  (1000.0*left_leg_Ty_Nm + ankleLength*left_leg_fx_N)/left_leg_fz_N;
		double left_zmpy_sensed_by_ft_mm  = -(-1000.0*left_leg_Tx_Nm + ankleLength*left_leg_fy_N)/left_leg_fz_N;

		double zmpfz_sensed_by_ft_N  = abs(right_leg_fz_N) + abs(left_leg_fz_N);
		double zmpx_sensed_by_ft_mm  = (right_zmpx_sensed_by_ft_mm + epr.y - Kinematics::LEG_SIDE_OFFSET/2.0)*right_leg_fz_N + (left_zmpx_sensed_by_ft_mm + epl.y + Kinematics::LEG_SIDE_OFFSET/2.0) /(right_leg_fz_N + left_leg_fz_N);
		double zmpy_sensed_by_ft_mm  = (right_zmpy_sensed_by_ft_mm + epr.x)*right_leg_fz_N + (left_zmpy_sensed_by_ft_mm + epl.x) / (right_leg_fz_N + left_leg_fz_N);

		double gyro_roll_rad_per_sec  =  MotionStatus::RL_GYRO * 27.925268 / 512.0;
		double gyro_pitch_rad_per_sec =  MotionStatus::FB_GYRO * 27.925268 / 512.0;

		double gyro_pitch_error_rad_per_sec = gyro_pitch_rad_per_sec;
		double gyro_roll_error_rad_per_sec = gyro_roll_rad_per_sec;

		double iu_roll_rad = MotionStatus::EulerAngleX;
		double iu_pitch_rad = MotionStatus::EulerAngleY;

		double iu_roll_error_rad = m_iu_roll_init_rad - iu_roll_rad;
		double iu_pitch_error_rad = m_iu_pitch_init_rad - iu_pitch_rad;


		cob_x_adjustment_mm = (IMU_GYRO_GAIN_RATIO*iu_pitch_error_rad+gyro_pitch_error_rad_per_sec) * BALANCE_X_GAIN;
		cob_y_adjustment_mm = (IMU_GYRO_GAIN_RATIO*iu_roll_error_rad+gyro_roll_error_rad_per_sec) * BALANCE_Y_GAIN;
		switch(m_Balancing_Index){
		case 0:
			if(DEBUG_PRINT)
				fprintf(stderr, "DSP : START\n");
			foot_r_roll_adjustment_rad  = foot_r_roll_landing_offset_rad + (IMU_GYRO_GAIN_RATIO*iu_roll_error_rad+gyro_roll_error_rad_per_sec) * BALANCE_ROLL_GAIN;
			foot_r_pitch_adjustment_rad = foot_r_pitch_landing_offset_rad + (IMU_GYRO_GAIN_RATIO*iu_pitch_error_rad+gyro_pitch_error_rad_per_sec) * BALANCE_PITCH_GAIN;
			foot_l_roll_adjustment_rad  = foot_l_roll_landing_offset_rad + (IMU_GYRO_GAIN_RATIO*iu_roll_error_rad+gyro_roll_error_rad_per_sec) * BALANCE_ROLL_GAIN;
			foot_l_pitch_adjustment_rad = foot_l_pitch_landing_offset_rad + (IMU_GYRO_GAIN_RATIO*iu_pitch_error_rad+gyro_pitch_error_rad_per_sec) * BALANCE_PITCH_GAIN;
			foot_landing_detection_time_sec = 0;
			break;
		case 1:
			if(DEBUG_PRINT)
				fprintf(stderr, "DSP : R--O->L\n");
			//            foot_r_roll_adjustment_rad  = 0;
			//            foot_r_pitch_adjustment_rad = 0;
			//            foot_l_roll_adjustment_rad  = 0;
			//            foot_l_pitch_adjustment_rad = 0;
			foot_r_roll_adjustment_rad  = foot_r_roll_landing_offset_rad + (IMU_GYRO_GAIN_RATIO*iu_roll_error_rad+gyro_roll_error_rad_per_sec) * BALANCE_ROLL_GAIN;
			foot_r_pitch_adjustment_rad = foot_r_pitch_landing_offset_rad + (IMU_GYRO_GAIN_RATIO*iu_pitch_error_rad+gyro_pitch_error_rad_per_sec) * BALANCE_PITCH_GAIN;
			foot_l_roll_adjustment_rad  = foot_l_roll_landing_offset_rad + (IMU_GYRO_GAIN_RATIO*iu_roll_error_rad+gyro_roll_error_rad_per_sec) * BALANCE_ROLL_GAIN;
			foot_l_pitch_adjustment_rad = foot_l_pitch_landing_offset_rad + (IMU_GYRO_GAIN_RATIO*iu_pitch_error_rad+gyro_pitch_error_rad_per_sec) * BALANCE_PITCH_GAIN;
			break;
		case 2:
			if(DEBUG_PRINT)
				fprintf(stderr, "SSP : L_BALANCING1\n");
			foot_r_roll_landing_offset_rad = FOOT_LANDING_OFFSET_GAIN*iu_roll_error_rad;
			foot_r_pitch_landing_offset_rad = FOOT_LANDING_OFFSET_GAIN*iu_pitch_error_rad;
			foot_r_roll_adjustment_rad  = foot_r_roll_landing_offset_rad + 1*(IMU_GYRO_GAIN_RATIO*iu_roll_error_rad+gyro_roll_error_rad_per_sec) * BALANCE_ROLL_GAIN;
			foot_r_pitch_adjustment_rad = foot_r_pitch_landing_offset_rad + 1*(IMU_GYRO_GAIN_RATIO*iu_pitch_error_rad+gyro_pitch_error_rad_per_sec) * BALANCE_PITCH_GAIN;
			foot_l_roll_adjustment_rad  = foot_l_roll_landing_offset_rad + (IMU_GYRO_GAIN_RATIO*iu_roll_error_rad+gyro_roll_error_rad_per_sec) * BALANCE_ROLL_GAIN;
			foot_l_pitch_adjustment_rad = foot_l_pitch_landing_offset_rad + (IMU_GYRO_GAIN_RATIO*iu_pitch_error_rad+gyro_pitch_error_rad_per_sec) * BALANCE_PITCH_GAIN;
			foot_landing_detection_time_sec = 0;
			break;
		case 3:
			if(DEBUG_PRINT)
				fprintf(stderr, "SSP : L_BALANCING2\n");
			foot_r_roll_landing_offset_rad = FOOT_LANDING_OFFSET_GAIN*iu_roll_error_rad;
			foot_r_pitch_landing_offset_rad = FOOT_LANDING_OFFSET_GAIN*iu_pitch_error_rad;
			foot_r_roll_adjustment_rad  = foot_r_roll_landing_offset_rad + 1*(IMU_GYRO_GAIN_RATIO*iu_roll_error_rad+gyro_roll_error_rad_per_sec) * BALANCE_ROLL_GAIN;
			foot_r_pitch_adjustment_rad = foot_r_pitch_landing_offset_rad + 1*(IMU_GYRO_GAIN_RATIO*iu_pitch_error_rad+gyro_pitch_error_rad_per_sec) * BALANCE_PITCH_GAIN;
			foot_l_roll_adjustment_rad  = foot_l_roll_landing_offset_rad + (IMU_GYRO_GAIN_RATIO*iu_roll_error_rad+gyro_roll_error_rad_per_sec) * BALANCE_ROLL_GAIN;
			foot_l_pitch_adjustment_rad = foot_l_pitch_landing_offset_rad + (IMU_GYRO_GAIN_RATIO*iu_pitch_error_rad+gyro_pitch_error_rad_per_sec) * BALANCE_PITCH_GAIN;

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
			//
			//            bIslanded = true;
			//            for(int i = 0; i < FT_SENSOR_WINDOW_SIZE ; i++ )
			//            {
			//            	bIslanded &= (m_right_leg_ft_fz_N_array[i] > FOOT_LANDING_DETECT_N);
			//            }
			//            if(DEBUG_PRINT)
			//            {
			//            	fprintf(stderr, "rightFZ : %f %f %f\n", m_right_leg_ft_fz_N_array[0], m_right_leg_ft_fz_N_array[1], m_right_leg_ft_fz_N_array[2]);
			//            }
			//            if (false && m_Balancing_Index == 4 && bIslanded == false)
			//            {
			//            	if(DEBUG_PRINT)
			//            		fprintf(stderr, "Wait DSP : R--O<-L !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
			//            	if (foot_landing_detection_time_sec < FOOT_LANDING_DETECTION_TIME_MAX_SEC) {
			//            		data_index = data_index-1;
			//            		foot_landing_detection_time_sec = foot_landing_detection_time_sec + SYSTEM_CONTROL_UNIT_TIME_SEC;
			//            	}
			//            	else
			//            		if(DEBUG_PRINT){
			//            			fprintf(stderr, "Stop waiting DSP : R--O<-L ##################################\n");
			//            		}
			//            }
			//            if(false && m_Balancing_Index != 4 && bIslanded == true)
			//            {
			//            	if(DEBUG_PRINT)
			//            		fprintf(stderr, "Jump DSP : R--O<-L ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ \n");
			//
			//            	for(int i = data_index + 1 ; i< data_size ; i++)
			//            	{
			//            		if (m_Balancing_Index != 3)
			//            		{
			//            			data_index = i;
			//            			break;
			//            		}
			//            	}
			//            }

			break;
		case 4:
			if(DEBUG_PRINT)
				fprintf(stderr, "DSP : R--O<-L\n");

			foot_r_roll_adjustment_rad  = foot_r_roll_landing_offset_rad + (IMU_GYRO_GAIN_RATIO*iu_roll_error_rad+gyro_roll_error_rad_per_sec) * BALANCE_ROLL_GAIN;
			foot_r_pitch_adjustment_rad = foot_r_pitch_landing_offset_rad + (IMU_GYRO_GAIN_RATIO*iu_pitch_error_rad+gyro_pitch_error_rad_per_sec) * BALANCE_PITCH_GAIN;
			foot_l_roll_adjustment_rad  = foot_l_roll_landing_offset_rad + (IMU_GYRO_GAIN_RATIO*iu_roll_error_rad+gyro_roll_error_rad_per_sec) * BALANCE_ROLL_GAIN;
			foot_l_pitch_adjustment_rad = foot_l_pitch_landing_offset_rad + (IMU_GYRO_GAIN_RATIO*iu_pitch_error_rad+gyro_pitch_error_rad_per_sec) * BALANCE_PITCH_GAIN;
			foot_landing_detection_time_sec = 0;
			break;
		case 5:
			if(DEBUG_PRINT)
				fprintf(stderr, "DSP : R<-O--L\n");

			foot_r_roll_adjustment_rad  = foot_r_roll_landing_offset_rad + (IMU_GYRO_GAIN_RATIO*iu_roll_error_rad+gyro_roll_error_rad_per_sec) * BALANCE_ROLL_GAIN;
			foot_r_pitch_adjustment_rad = foot_r_pitch_landing_offset_rad + (IMU_GYRO_GAIN_RATIO*iu_pitch_error_rad+gyro_pitch_error_rad_per_sec) * BALANCE_PITCH_GAIN;
			foot_l_roll_adjustment_rad  = foot_l_roll_landing_offset_rad + (IMU_GYRO_GAIN_RATIO*iu_roll_error_rad+gyro_roll_error_rad_per_sec) * BALANCE_ROLL_GAIN;
			foot_l_pitch_adjustment_rad = foot_l_pitch_landing_offset_rad + (IMU_GYRO_GAIN_RATIO*iu_pitch_error_rad+gyro_pitch_error_rad_per_sec) * BALANCE_PITCH_GAIN;
			break;
		case 6:
			if(DEBUG_PRINT)
				fprintf(stderr, "SSP : R_BALANCING1\n");
			foot_l_roll_landing_offset_rad = FOOT_LANDING_OFFSET_GAIN*iu_roll_error_rad;
			foot_l_pitch_landing_offset_rad = FOOT_LANDING_OFFSET_GAIN*iu_pitch_error_rad;
			foot_r_roll_adjustment_rad  = foot_r_roll_landing_offset_rad + (IMU_GYRO_GAIN_RATIO*iu_roll_error_rad+gyro_roll_error_rad_per_sec) * BALANCE_ROLL_GAIN;
			foot_r_pitch_adjustment_rad = foot_r_pitch_landing_offset_rad + (IMU_GYRO_GAIN_RATIO*iu_pitch_error_rad+gyro_pitch_error_rad_per_sec) * BALANCE_PITCH_GAIN;
			foot_l_roll_adjustment_rad  = foot_l_roll_landing_offset_rad + 1*(IMU_GYRO_GAIN_RATIO*iu_roll_error_rad+gyro_roll_error_rad_per_sec) * BALANCE_ROLL_GAIN;
			foot_l_pitch_adjustment_rad = foot_l_pitch_landing_offset_rad + 1*(IMU_GYRO_GAIN_RATIO*iu_pitch_error_rad+gyro_pitch_error_rad_per_sec) * BALANCE_PITCH_GAIN;
			foot_landing_detection_time_sec = 0;
			break;
		case 7:
			if(DEBUG_PRINT)
				fprintf(stderr, "SSP : R_BALANCING2\n");
			foot_l_roll_landing_offset_rad = FOOT_LANDING_OFFSET_GAIN*iu_roll_error_rad;
			foot_l_pitch_landing_offset_rad = FOOT_LANDING_OFFSET_GAIN*iu_pitch_error_rad;
			foot_r_roll_adjustment_rad  = foot_r_roll_landing_offset_rad + (IMU_GYRO_GAIN_RATIO*iu_roll_error_rad+gyro_roll_error_rad_per_sec) * BALANCE_ROLL_GAIN;
			foot_r_pitch_adjustment_rad = foot_r_pitch_landing_offset_rad + (IMU_GYRO_GAIN_RATIO*iu_pitch_error_rad+gyro_pitch_error_rad_per_sec) * BALANCE_PITCH_GAIN;
			foot_l_roll_adjustment_rad  = foot_l_roll_landing_offset_rad + 1*(IMU_GYRO_GAIN_RATIO*iu_roll_error_rad+gyro_roll_error_rad_per_sec) * BALANCE_ROLL_GAIN;
			foot_l_pitch_adjustment_rad = foot_l_pitch_landing_offset_rad + 1*(IMU_GYRO_GAIN_RATIO*iu_pitch_error_rad+gyro_pitch_error_rad_per_sec) * BALANCE_PITCH_GAIN;

			bIslanded = true;
			for(int i = 0; i < FT_SENSOR_WINDOW_SIZE ; i++ )
			{
				bIslanded &= (m_left_leg_ft_fz_N_array[i] > FOOT_LANDING_DETECT_N);
			}
			if(DEBUG_PRINT)
			{
				fprintf(stderr, "leftFZ : %f %f %f\n", m_left_leg_ft_fz_N_array[0], m_left_leg_ft_fz_N_array[1], m_left_leg_ft_fz_N_array[2]);
			}
			//            if (false && m_Balancing_Index == 8 && bIslanded == false) {
			//            	if(DEBUG_PRINT)
			//            		fprintf(stderr, "Wait DSP : R->O--L !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
			//            	if (foot_landing_detection_time_sec < FOOT_LANDING_DETECTION_TIME_MAX_SEC) {
			//            		data_index = data_index-1;
			//            		foot_landing_detection_time_sec = foot_landing_detection_time_sec + SYSTEM_CONTROL_UNIT_TIME_SEC;
			//            	}
			//            	else
			//            		fprintf(stderr, "Stop waiting DSP : R->O--L ##################################\n");
			//            }
			//
			//            if (false && BalancingIdxData[data_index+1] != 8 && bIslanded == true) {
			//            	if(DEBUG_PRINT)
			//            		fprintf(stderr, "Jump DSP : R->O--L ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~");
			//            	for(int i = data_index + 1 ; i< data_size ; i++) {
			//            		if (BalancingIdxData[i] != 7){
			//            			data_index = i;
			//            			break;
			//            		}
			//            	}
			//            }

			break;
		case 8:
			if(DEBUG_PRINT)
				fprintf(stderr, "DSP : R->O--L");

			foot_r_roll_adjustment_rad  = foot_r_roll_landing_offset_rad + (IMU_GYRO_GAIN_RATIO*iu_roll_error_rad+gyro_roll_error_rad_per_sec) * BALANCE_ROLL_GAIN;
			foot_r_pitch_adjustment_rad = foot_r_pitch_landing_offset_rad + (IMU_GYRO_GAIN_RATIO*iu_pitch_error_rad+gyro_pitch_error_rad_per_sec) * BALANCE_PITCH_GAIN;
			foot_l_roll_adjustment_rad  = foot_l_roll_landing_offset_rad + (IMU_GYRO_GAIN_RATIO*iu_roll_error_rad+gyro_roll_error_rad_per_sec) * BALANCE_ROLL_GAIN;
			foot_l_pitch_adjustment_rad = foot_l_pitch_landing_offset_rad + (IMU_GYRO_GAIN_RATIO*iu_pitch_error_rad+gyro_pitch_error_rad_per_sec) * BALANCE_PITCH_GAIN;
			break;
		case 9:
			if(DEBUG_PRINT)
				fprintf(stderr, "DSP : END");

			foot_r_roll_adjustment_rad  = foot_r_roll_landing_offset_rad + (IMU_GYRO_GAIN_RATIO*iu_roll_error_rad+gyro_roll_error_rad_per_sec) * BALANCE_ROLL_GAIN;
			foot_r_pitch_adjustment_rad = foot_r_pitch_landing_offset_rad + (IMU_GYRO_GAIN_RATIO*iu_pitch_error_rad+gyro_pitch_error_rad_per_sec) * BALANCE_PITCH_GAIN;
			foot_l_roll_adjustment_rad  = foot_l_roll_landing_offset_rad + (IMU_GYRO_GAIN_RATIO*iu_roll_error_rad+gyro_roll_error_rad_per_sec) * BALANCE_ROLL_GAIN;
			foot_l_pitch_adjustment_rad = foot_l_pitch_landing_offset_rad + (IMU_GYRO_GAIN_RATIO*iu_pitch_error_rad+gyro_pitch_error_rad_per_sec) * BALANCE_PITCH_GAIN;
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
			fprintf(stderr, " : %f %f %f %f %f %f %f %f\n", cob_x_adjustment_mm, cob_y_adjustment_mm, foot_r_roll_adjustment_rad*180.0/PI, foot_r_pitch_adjustment_rad*180.0/PI, foot_l_roll_adjustment_rad*180.0/PI, foot_l_pitch_adjustment_rad*180.0/PI,  right_leg_fz_N,  left_leg_fz_N);

		epr.x -= cob_x_adjustment_mm;
		epr.y -= cob_y_adjustment_mm;

		epr.roll += foot_r_roll_adjustment_rad;
		epr.pitch += foot_r_pitch_adjustment_rad;

		epl.x -= cob_x_adjustment_mm;
		epl.y -= cob_y_adjustment_mm;

		epl.roll += foot_l_roll_adjustment_rad;
		epl.pitch += foot_l_pitch_adjustment_rad;
	}
#endif

	if(computeIK(&angle[0], epr.x, epr.y, epr.z + Kinematics::LEG_LENGTH, epr.roll, epr.pitch, epr.yaw) == false)	{
		printf("IK not Solved\n");
		return;
	}

	if(computeIK(&angle[6], epl.x, epl.y, epl.z + Kinematics::LEG_LENGTH, epl.roll, epl.pitch, epl.yaw) == false)	{
		printf("IK not Solved\n");
		return;
	}

	for(int idx = 0; idx < 6; idx++)
	{
		angle[idx] = (double)dir_output[idx]*angle[idx]*180.0/PI + InitAngle[idx];
		angle[idx+6] = (double)dir_output[idx+6]*angle[idx+6]*180.0/PI + InitAngle[idx+6];
	}

	angle[12] = (double)dir_output[12]*(epr.x - epl.x)*shoulder_swing_gain + InitAngle[12];
	angle[13] = (double)dir_output[13]*(epl.x - epr.x)*shoulder_swing_gain + InitAngle[13];
	angle[14] = (double)dir_output[14]*(epr.x - epl.x)*elbow_swing_gain    + InitAngle[14];
	angle[15] = (double)dir_output[15]*(epl.x - epr.x)*elbow_swing_gain    + InitAngle[15];


	if(m_Real_Running == true)
	{
		m_play_idx = (int) m_dplay_idx;
		if(m_play_idx >= m_PatternDataSize-1)
		{
			m_play_idx = m_PatternDataSize-1;
			m_dplay_idx = m_play_idx;
			m_ReferenceGtoLeftFootPosition = GetPose3DfromTransformMatrix(matGtoLFp);
			m_ReferenceGtoRightFootPosition = GetPose3DfromTransformMatrix(matGtoRFp);
			m_ReferenceGtoBodyPosition = GetPose3DfromTransformMatrix(matGtoCOBp);
			m_Real_Running = false;
			//printf("Body  : %f %f %f %f %f %f\n", m_ReferenceGtoBodyPosition.x, m_ReferenceGtoBodyPosition.y, m_ReferenceGtoBodyPosition.z, m_ReferenceGtoBodyPosition.roll, m_ReferenceGtoBodyPosition.pitch, m_ReferenceGtoBodyPosition.yaw);
			//			printf("Rfoot : %f %f %f %f %f %f\n", m_ReferenceGtoRightFootPosition.x, m_ReferenceGtoRightFootPosition.y, m_ReferenceGtoRightFootPosition.z, m_ReferenceGtoRightFootPosition.roll, m_ReferenceGtoRightFootPosition.pitch, m_ReferenceGtoRightFootPosition.yaw);
			//			printf("Lfoot : %f %f %f %f %f %f\n", m_ReferenceGtoLeftFootPosition.x, m_ReferenceGtoLeftFootPosition.y, m_ReferenceGtoLeftFootPosition.z, m_ReferenceGtoLeftFootPosition.roll, m_ReferenceGtoLeftFootPosition.pitch, m_ReferenceGtoLeftFootPosition.yaw);
		}
	}

#ifdef WEBOT_SIMULATION
	for(int idx = 0; idx < 16; idx++) {
		Webot_Rad[idx] = angle[idx]*PI/180.0;
		//printf("%f  ", Webot_Rad[idx]);
	}
	//printf("\n");
#else
	//	for(int idx = 0; idx < 16; idx++)
	//		outValue[idx] = (int)(angle[idx]*251000.0/180.0);
	for(int idx = 0; idx < 16; idx++)
	{
		//		double offset = (double)dir[i] * angle[i] * (g_pro54->MAX_VALUE)/180.0;
		//        outValue[i] = g_pro54->Angle2Value(initAngle[i]) + (int)offset;
		outValue[idx] = g_pro54->Angle2Value(angle[idx]);
	}

	outValue[2] -= (double)dir_output[2] * HIP_PITCH_OFFSET * (g_pro54->MAX_VALUE)/180.0;
	outValue[8] -= (double)dir_output[8] * HIP_PITCH_OFFSET * (g_pro54->MAX_VALUE)/180.0;

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
		else if( id == 17) {
			m_RobotInfo[jointIndex].m_Value = outValue[1];
			m_RobotInfo[jointIndex].m_Pgain = P_GAIN;
			m_RobotInfo[jointIndex].m_Igain = I_GAIN;
			m_RobotInfo[jointIndex].m_Dgain = D_GAIN;
		}
		else if( id == 19) {
			m_RobotInfo[jointIndex].m_Value = outValue[2];
			m_RobotInfo[jointIndex].m_Pgain = P_GAIN;
			m_RobotInfo[jointIndex].m_Igain = I_GAIN;
			m_RobotInfo[jointIndex].m_Dgain = D_GAIN;
		}
		else if(id == 21) {
			m_RobotInfo[jointIndex].m_Value = outValue[3];
			m_RobotInfo[jointIndex].m_Pgain = P_GAIN;
			m_RobotInfo[jointIndex].m_Igain = I_GAIN;
			m_RobotInfo[jointIndex].m_Dgain = D_GAIN;
		}
		else if(id == 23) {
			m_RobotInfo[jointIndex].m_Value = outValue[4];
			m_RobotInfo[jointIndex].m_Pgain = P_GAIN;
			m_RobotInfo[jointIndex].m_Igain = I_GAIN;
			m_RobotInfo[jointIndex].m_Dgain = D_GAIN;
		}
		else if(id == 25) {
			m_RobotInfo[jointIndex].m_Value = outValue[5];
			m_RobotInfo[jointIndex].m_Pgain = P_GAIN;
			m_RobotInfo[jointIndex].m_Igain = I_GAIN;
			m_RobotInfo[jointIndex].m_Dgain = D_GAIN;
		}
		else if(id == 16) {
			m_RobotInfo[jointIndex].m_Value = outValue[6];
			m_RobotInfo[jointIndex].m_Pgain = P_GAIN;
			m_RobotInfo[jointIndex].m_Igain = I_GAIN;
			m_RobotInfo[jointIndex].m_Dgain = D_GAIN;
		}
		else if( id == 18) {
			m_RobotInfo[jointIndex].m_Value = outValue[7];
			m_RobotInfo[jointIndex].m_Pgain = P_GAIN;
			m_RobotInfo[jointIndex].m_Igain = I_GAIN;
			m_RobotInfo[jointIndex].m_Dgain = D_GAIN;
		}
		else if( id == 20) {
			m_RobotInfo[jointIndex].m_Value = outValue[8];
			m_RobotInfo[jointIndex].m_Pgain = P_GAIN;
			m_RobotInfo[jointIndex].m_Igain = I_GAIN;
			m_RobotInfo[jointIndex].m_Dgain = D_GAIN;
		}
		else if(id == 22) {
			m_RobotInfo[jointIndex].m_Value = outValue[9];
			m_RobotInfo[jointIndex].m_Pgain = P_GAIN;
			m_RobotInfo[jointIndex].m_Igain = I_GAIN;
			m_RobotInfo[jointIndex].m_Dgain = D_GAIN;
		}
		else if(id == 24) {
			m_RobotInfo[jointIndex].m_Value = outValue[10];
			m_RobotInfo[jointIndex].m_Pgain = P_GAIN;
			m_RobotInfo[jointIndex].m_Igain = I_GAIN;
			m_RobotInfo[jointIndex].m_Dgain = D_GAIN;
		}
		else if(id == 26) {
			m_RobotInfo[jointIndex].m_Value = outValue[11];
			m_RobotInfo[jointIndex].m_Pgain = P_GAIN;
			m_RobotInfo[jointIndex].m_Igain = I_GAIN;
			m_RobotInfo[jointIndex].m_Dgain = D_GAIN;
		}
		else if(id == 1) {
			m_RobotInfo[jointIndex].m_Value = outValue[12];
			m_RobotInfo[jointIndex].m_Pgain = P_GAIN;
			m_RobotInfo[jointIndex].m_Igain = I_GAIN;
			m_RobotInfo[jointIndex].m_Dgain = D_GAIN;
		}
		else if(id == 2) {
			m_RobotInfo[jointIndex].m_Value = outValue[13];
			m_RobotInfo[jointIndex].m_Pgain = P_GAIN;
			m_RobotInfo[jointIndex].m_Igain = I_GAIN;
			m_RobotInfo[jointIndex].m_Dgain = D_GAIN;
		}
		else if(id == 7) {
			m_RobotInfo[jointIndex].m_Value = outValue[14];
			m_RobotInfo[jointIndex].m_Pgain = P_GAIN;
			m_RobotInfo[jointIndex].m_Igain = I_GAIN;
			m_RobotInfo[jointIndex].m_Dgain = D_GAIN;
		}
		else if(id == 8) {
			m_RobotInfo[jointIndex].m_Value = outValue[15];
			m_RobotInfo[jointIndex].m_Pgain = P_GAIN;
			m_RobotInfo[jointIndex].m_Igain = I_GAIN;
			m_RobotInfo[jointIndex].m_Dgain = D_GAIN;
		}
		else
			continue;
	}

#endif
}

bool RecursiveWalking::computeIK(double *out, double x, double y, double z, double a, double b, double c)
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
	if(std::isnan(_Acos) == 1)
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
	if(std::isnan(_Acos) == 1)
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
	if(std::isinf(_Atan) == 1)
		return false;
	*(out) = _Atan;


	// Get Hip Roll
	_Atan = atan2(Tac.m[9], -Tac.m[1] * sin(*(out)) + Tac.m[5] * cos(*(out)));
	if(std::isinf(_Atan) == 1)
		return false;
	*(out + 1) = _Atan;


	// Get Hip Pitch and Ankle Pitch
	_Atan = atan2(Tac.m[2] * cos(*(out)) + Tac.m[6] * sin(*(out)), Tac.m[0] * cos(*(out)) + Tac.m[4] * sin(*(out)));
	if(std::isinf(_Atan) == 1)
		return false;
	_theta = _Atan;
	_k = sin(*(out + 3)) * CALF_LENGTH;
	_l = -THIGH_LENGTH - cos(*(out + 3)) * CALF_LENGTH;
	_m = cos(*(out)) * vec.X + sin(*(out)) * vec.Y;
	_n = cos(*(out + 1)) * vec.Z + sin(*(out)) * sin(*(out + 1)) * vec.X - cos(*(out)) * sin(*(out + 1)) * vec.Y;
	_s = (_k * _n + _l * _m) / (_k * _k + _l * _l);
	_c = (_n - _k * _s) / _l;
	_Atan = atan2(_s, _c);
	if(std::isinf(_Atan) == 1)
		return false;
	*(out + 2) = _Atan;
	*(out + 4) = _theta - *(out + 3) - *(out + 2);

	return true;
}

