#ifdef WEBOT_SIMULATION
#include "Vector.h"
#include "Matrix.h"
#include "Kinematics.h"
#include "Dynamics.h"
#include "PreviewControlWalking.h"
#else if
#include "framework/math/vector.h"
#include "framework/math/matrix.h"
#include "framework/motion/kinematics.h"
#include "framework/motion/dynamics.h"
#include "framework/motion/PRO54.h"
#include "framework/motion/modules/preview_control_walking.h"
#endif

using namespace Thor;

#define PI (3.141592653589793)
#define G  (9810) // mm/s^2(= 9.81m/s^2 *1000mm/1m)

#define NO_STEP_IDX  (-1)

#define FT_SENSOR_WINDOW_SIZE 3

static PRO54 *g_pro54 = new PRO54();

bool step_data_locker = false;

PreviewControlWalking* PreviewControlWalking::m_UniqueInstance = new PreviewControlWalking();

PreviewControlWalking::PreviewControlWalking()
{
	//std::vector<StepData> m_StepData;

	uID = "PreviewControlWalking";
	m_Real_Running = false;  m_Ctrl_Running = false;

	m_PresentRightFootPosition = { 0, -95,   0, 0, 0, 0};
	m_PresentLeftFootPosition  = { 0,  95,   0, 0, 0, 0};
	m_PresentBodyPosition      = { 0,   0, 670, 0, 0, 0};

	m_ReferenceRightFootPosition = { 0, -95,   0, 0, 0, 0};
	m_ReferenceLeftFootPosition  = { 0,  95,   0, 0, 0, 0};
	m_ReferenceBodyPosition      = { 0,   0, 670, 0, 0, 0};


	m_Time = 0; m_WalkingTime = 0; m_ReferenceTime = 0;
	m_PresentStepNum = 0;
	m_PreviousStepNum = 0;
	m_Balancing_Index = 0;

	m_PeriodTime = 848;
	m_DSP_Ratio = 0.2;
	m_SSP_Ratio = 1 - m_DSP_Ratio;
	m_Foot_Move_PeriodTime = m_SSP_Ratio*m_PeriodTime;
	m_Body_Move_PeriodTime = m_PeriodTime;

	m_SSP_Time = m_Foot_Move_PeriodTime;
	m_SSP_Time_Start = m_DSP_Ratio*m_PeriodTime/2.0;
	m_SSP_Time_End = (1 + m_SSP_Ratio)*m_PeriodTime / 2.0;


	//Initial Pose
	X_Offset = 0.0; Y_OFfset = 46.0; Z_Offset = Kinematics::LEG_LENGTH - 670.0;
	A_Offset = 0.0;	B_Offset = 0.0;  C_Offset = 0.0;

	m_X_ZMP_Init = -14.251206; m_X_ZMP_CenterShift = 0.0;
	m_Y_ZMP_Convergence = 0.0; m_Y_ZMP_CenterShift = 0.0;

	//m_X_ZMP_Init = -35.977034;

	m_PreivewTime = 2.0; //(sec)
	m_DynamicsFilterTime = 1.0; //(sec)

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

	BALANCE_ENABLE = true;
	DEBUG_PRINT = false;


	m_right_leg_ft_fz_N_array = 0;
	m_left_leg_ft_fz_N_array = 0;
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

	m_iu_roll_init_rad = 0;
	m_iu_pitch_init_rad = 0;
	m_iu_yaw_init_rad = 0;

	P_GAIN = 64;
	I_GAIN = 0;
	D_GAIN = 0;

	m_right_leg_ft_fz_N_array = 0;
	m_left_leg_ft_fz_N_array = 0;

	FILTERING_ENABLE = false;

}

PreviewControlWalking::~PreviewControlWalking()
{   }

void PreviewControlWalking::Initialize()
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
#endif


	//////Initialize parameter for preview control
	double t = TIME_UNIT/1000.0;
	double z_COM_mm = Kinematics::LEG_LENGTH - Z_Offset;
	A.resize(3,3); b.resize(3,1); c.resize(1,3);
	A << 1,  t, t*t/2.0,
			0,  1,   t,
			0,  0,   1;
	b(0,0) = t*t*t/6.0;
	b(1,0) =   t*t/2.0;
	b(2,0) =	 t;

	c(0,0) = 1; c(0,1) = 0; c(0,2) = -z_COM_mm/G;

	double Q_e = 1.0, R = 0.000001;//1.0e-6;
	m_PreviewSize = (int)(m_PreivewTime/t);
	m_DFilterSize = (int)(m_DynamicsFilterTime/t);

	f_Preview.resize(1, m_PreviewSize);
	f_Dfilter.resize(1, m_DFilterSize);

	K.resize(1,3);
	K <<741.039567347343,	401.126578170676,	57.9541468439414;

	P.resize(3,3);
	P << 67.6628138101049,	  18.0423742355632, 	0.0957907173930850,
			18.0423742355632,	   4.97721070978477,	0.0698572131186252,
			0.0957907173930850,  0.0698572131186252,	0.0121865166724453;
	double _temp;
	matd _temp1, _temp2, _temp3(3,3);

	_temp = R + (b.transpose() * P * b)(0, 0);
	_temp1 = (1.0/_temp)*b.transpose();
	_temp2.setIdentity(3, 3);
	_temp3 = (A - b*K);
	_temp3.transposeInPlace();

	for(int idx_for_f_calc = 0; idx_for_f_calc < m_PreviewSize; idx_for_f_calc++)
	{
		f_Preview(0, idx_for_f_calc) = (_temp1*_temp2*c.transpose()*Q_e)(0, 0); //(R+b'*P*b)\b'*(((A - b*K)')^(i-1))*c'*Q_e;
		_temp2 = _temp2*_temp3; //((A - b*K)')^i;
	}

	_temp2.setIdentity(3, 3);
	for(int idx_for_f_calc = 0; idx_for_f_calc < m_DFilterSize; idx_for_f_calc++)
	{
		f_Dfilter(0, idx_for_f_calc) = (_temp1*_temp2*c.transpose()*Q_e)(0, 0);
		_temp2 = _temp2*_temp3;
	}

	std::cout << f_Preview << std::endl;
	std::cout << f_Dfilter << std::endl;

	x_LIPM.resize(3, m_DFilterSize);		y_LIPM.resize(3, m_DFilterSize);
	x_MPMM_pre.resize(3, m_DFilterSize);	y_MPMM_pre.resize(3, m_DFilterSize);
	//x_MPMM_post.resize(3, m_DFilterSize);	y_MPMM_post.resize(3, m_DFilterSize);
	x_delta.resize(3, m_DFilterSize);		y_delta.resize(3, m_DFilterSize);;


	m_ZMP_Reference_X.resize(m_PreviewSize + m_DFilterSize, 1); m_ZMP_Reference_Y.resize(m_PreviewSize + m_DFilterSize, 1);
	m_ZMP_Generated_by_LIPM_X.resize(m_DFilterSize, 1);			m_ZMP_Generated_by_LIPM_Y.resize(m_DFilterSize, 1);
	m_ZMP_Calculated_by_MPMM_X_pre.resize(m_DFilterSize, 1); 	m_ZMP_Calculated_by_MPMM_Y_pre.resize(m_DFilterSize, 1);
	m_ZMP_Calculated_by_MPMM_X_post.resize(m_DFilterSize, 1);	m_ZMP_Calculated_by_MPMM_Y_post.resize(m_DFilterSize, 1);

	m_StepIdxData.resize(m_PreviewSize+m_DFilterSize);
	m_StepIdxData.fill(NO_STEP_IDX);

	m_matvGtoRF.resize(m_DFilterSize);
	m_matvGtoLF.resize(m_DFilterSize);
	m_matvGtoCOB.resize(m_DFilterSize);

	vecd vecTemp(3);
	vecTemp.fill(0);
	x_LIPM.col(0) = vecTemp; y_LIPM.col(0) = vecTemp;
	x_delta.col(0) = vecTemp; y_delta.col(0) = vecTemp;
	y_MPMM_pre.col(0) = vecTemp;

	vecTemp(0) = m_X_ZMP_Init;
	x_MPMM_pre.col(0) = vecTemp;

	matCOBtoG = GetTransformMatrixInverse(GetTranslationMatrix(0,0,670));
	matRHtoRF = matRHtoCOB*matCOBtoG*GetTranslationMatrix(0,-95,0);
	matLHtoLF = matLHtoCOB*matCOBtoG*GetTranslationMatrix(0, 95,0);

	std::cout<< matRHtoRF <<std::endl;
	Pose3D epr = GetPose3DfromTransformMatrix(matRHtoRF);
	Pose3D epl = GetPose3DfromTransformMatrix(matLHtoLF);

	double angle[12];
	double langle[6], rangle[6];
	if(computeIK(&angle[0], epr.x, epr.y, epr.z + Kinematics::LEG_LENGTH, epr.roll, epr.pitch, epr.yaw) == false)
	{
		printf("IKnotsolver\n");
		return;
	}

	if(computeIK(&angle[6], epl.x, epl.y, epl.z + Kinematics::LEG_LENGTH, epl.roll, epl.pitch, epl.yaw) == false)
	{
		printf("IKnotsolvel\n");
		return;
	}

	for(int idx = 0; idx < 6; idx++)
	{
		rangle[idx] = angle[idx]*dir[idx]*180.0/PI;
		langle[idx] = angle[idx+6]*dir[idx+6]*180.0/PI;
	}

	PData3 = GetTransformMatrixInverse(matCOBtoG)*GetCOMofMultiBody(langle, rangle, l_arm, r_arm);
	PData2 = PData3;
	PData1 = PData3;

	FilterdPData1 = FilterdPData2 = FilterdPData3 = PData3;


	if(	m_right_leg_ft_fz_N_array != 0)
		delete[] m_right_leg_ft_fz_N_array;
	if(m_left_leg_ft_fz_N_array != 0)
		delete[] m_left_leg_ft_fz_N_array;

	m_right_leg_ft_fz_N_array = new double[FT_SENSOR_WINDOW_SIZE];
	m_left_leg_ft_fz_N_array = new double[FT_SENSOR_WINDOW_SIZE];
}

bool PreviewControlWalking::IsRunning()
{
	return m_Real_Running;
}

void PreviewControlWalking::Start()
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

	m_Ctrl_Running = true;
	m_Real_Running = true;
}

void PreviewControlWalking::Stop()
{
	m_Ctrl_Running = false;
}

void PreviewControlWalking::AddStepData(StepData step_data)
{
	while(step_data_locker);

	step_data_locker = true;
	m_StepData.push_back(step_data);
	step_data_locker = false;
}

void PreviewControlWalking::SetInitAngleinRad(double roll_init_rad, double pitch_init_rad)
{
	m_iu_roll_init_rad = roll_init_rad;
	m_iu_pitch_init_rad = pitch_init_rad;
}

void PreviewControlWalking::SetSizeforPreviewControl(double previewTime_sec, double dynamicsFilterTime_sec)
{
	m_PreivewTime = previewTime_sec;
	m_DynamicsFilterTime = dynamicsFilterTime_sec;
}

void PreviewControlWalking::SetRefZMPDecisionParameter(double X_ZMP_CenterShift, double Y_ZMP_CenterShift, double Y_ZMP_Convergence)
{
	m_X_ZMP_CenterShift = X_ZMP_CenterShift;
	m_Y_ZMP_CenterShift = Y_ZMP_CenterShift;
	m_Y_ZMP_Convergence = Y_ZMP_Convergence;
}

double PreviewControlWalking::wsin(double time, double period, double period_shift, double mag, double mag_shift)
{
	return mag * sin(2 * PI / period * time - period_shift) + mag_shift;
}

double PreviewControlWalking::wsigmoid(double time, double period, double time_shift, double mag, double mag_shift, double sigmoid_ratio, double distortion_ratio)
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

void PreviewControlWalking::CalcStepIdxData()
{
	unsigned int step_idx;
	unsigned int step_data_size = m_StepData.size();
	if(m_StepData.size() == 0) {
		m_StepIdxData.fill(NO_STEP_IDX);
		return;
	}
	else {
		for(unsigned int idx = 0; idx < m_DFilterSize + m_PreviewSize;  idx++)
		{
			//Get STepIDx
			//�좎룞�쇿뜝占썲뜝�쒓낀�쇿뜝�숈삕�좎룞���좎룞�쇿뜝�숈삕�좑옙�겼뜝�숈삕 �좎떎�먯삕
			if(m_WalkingTime + (idx+1)*TIME_UNIT > m_StepData[step_data_size-1].TimeData.dAbsStepTime)
				m_StepIdxData(idx) = NO_STEP_IDX;
			else {
				//�좎룞�쇿뜝占썲뜝�숈삕�좑옙�좎떆怨ㅼ삕�좎룞�쇿뜝�숈삕 �좎룞�쇿뜝�숈삕�좑옙�좎룜��Step Idx�좎룞�쇿뜝�숈삕 �좎떎�먯삕�좎떬�먯삕.
				for(step_idx = 0; step_idx < step_data_size; step_idx++) {
					if(m_WalkingTime + (idx+1)*TIME_UNIT <= m_StepData[step_idx].TimeData.dAbsStepTime)
						break;
				}//for end
				m_StepIdxData(idx) = step_idx;
			}// if else end
			///////////
		}
	}
}

void PreviewControlWalking::CalcRefZMP()
{
	unsigned int ref_zmp_idx = 0;
	unsigned int start_idx = 0;
	int step_idx = 0;
	if(m_StepIdxData(ref_zmp_idx) == NO_STEP_IDX)
	{
		m_ZMP_Reference_X(ref_zmp_idx, 0) = (m_ReferenceRightFootPosition.x + m_ReferenceLeftFootPosition.x)*0.5;
		m_ZMP_Reference_Y(ref_zmp_idx, 0) = (m_ReferenceRightFootPosition.y + m_ReferenceLeftFootPosition.y)*0.5;
		start_idx = 1;
	}

	for(ref_zmp_idx = start_idx; ref_zmp_idx < m_PreviewSize + m_DFilterSize; ref_zmp_idx++)
	{
		step_idx = m_StepIdxData(ref_zmp_idx);
		if(step_idx == NO_STEP_IDX)
		{
			m_ZMP_Reference_X(ref_zmp_idx, 0) = m_ZMP_Reference_X(ref_zmp_idx - 1, 0);
			m_ZMP_Reference_Y(ref_zmp_idx, 0) = m_ZMP_Reference_Y(ref_zmp_idx - 1, 0);
		}
		else
		{
			if(m_StepData[step_idx].TimeData.bWalkingState == InWalking)
			{
				if( m_StepData[step_idx].PositionData.bMovingFoot == RFootMove ) {
					m_ZMP_Reference_X(ref_zmp_idx, 0) = m_StepData[step_idx].PositionData.stLeftFootPosition.x + m_X_ZMP_CenterShift;
					m_ZMP_Reference_Y(ref_zmp_idx, 0) = m_StepData[step_idx].PositionData.stLeftFootPosition.y + m_Y_ZMP_CenterShift + m_Y_ZMP_Convergence;

				}
				else if( m_StepData[step_idx].PositionData.bMovingFoot == LFootMove ) {
					m_ZMP_Reference_X(ref_zmp_idx, 0) = m_StepData[step_idx].PositionData.stRightFootPosition.x + m_X_ZMP_CenterShift;
					m_ZMP_Reference_Y(ref_zmp_idx, 0) = m_StepData[step_idx].PositionData.stRightFootPosition.y + m_Y_ZMP_CenterShift + m_Y_ZMP_Convergence;
				}
				else if( m_StepData[step_idx].PositionData.bMovingFoot == NFootMove ) {
					m_ZMP_Reference_X(ref_zmp_idx, 0) = (m_StepData[step_idx].PositionData.stLeftFootPosition.x + m_StepData[step_idx].PositionData.stRightFootPosition.x)*0.5;
					m_ZMP_Reference_Y(ref_zmp_idx, 0) = (m_StepData[step_idx].PositionData.stLeftFootPosition.y + m_StepData[step_idx].PositionData.stRightFootPosition.y)*0.5;
				}
				else {
					m_ZMP_Reference_X(ref_zmp_idx, 0) = (m_StepData[step_idx].PositionData.stLeftFootPosition.x + m_StepData[step_idx].PositionData.stRightFootPosition.x)*0.5;
					m_ZMP_Reference_Y(ref_zmp_idx, 0) = (m_StepData[step_idx].PositionData.stLeftFootPosition.y + m_StepData[step_idx].PositionData.stRightFootPosition.y)*0.5;
				}
			}
			else if(m_StepData[step_idx].TimeData.bWalkingState == InWalkingStarting) {
				m_ZMP_Reference_X(ref_zmp_idx, 0) = (m_StepData[step_idx].PositionData.stLeftFootPosition.x + m_StepData[step_idx].PositionData.stRightFootPosition.x)*0.5 + m_X_ZMP_Init;
				m_ZMP_Reference_Y(ref_zmp_idx, 0) = (m_StepData[step_idx].PositionData.stLeftFootPosition.y + m_StepData[step_idx].PositionData.stRightFootPosition.y)*0.5;
			}
			else if(m_StepData[step_idx].TimeData.bWalkingState == InWalkingEnding) {
				m_ZMP_Reference_X(ref_zmp_idx, 0) = (m_StepData[step_idx].PositionData.stLeftFootPosition.x + m_StepData[step_idx].PositionData.stRightFootPosition.x)*0.5 + m_X_ZMP_Init;
				m_ZMP_Reference_Y(ref_zmp_idx, 0) = (m_StepData[step_idx].PositionData.stLeftFootPosition.y + m_StepData[step_idx].PositionData.stRightFootPosition.y)*0.5;
			}
			else {
				m_ZMP_Reference_X(ref_zmp_idx, 0) = (m_StepData[step_idx].PositionData.stLeftFootPosition.x + m_StepData[step_idx].PositionData.stRightFootPosition.x)*0.5;
				m_ZMP_Reference_Y(ref_zmp_idx, 0) = (m_StepData[step_idx].PositionData.stLeftFootPosition.y + m_StepData[step_idx].PositionData.stRightFootPosition.y)*0.5;
			}
		}
	}
}

void PreviewControlWalking::CalcEndPointData()
{
	unsigned int ep_idx = 0;
	unsigned int start_idx = 0;
	int step_idx = m_StepIdxData(ep_idx);

	int time, ref_time = m_ReferenceTime;
	static double period_time, dsp_ratio, ssp_ratio, foot_move_period_time, body_move_period_time, ssp_time_start, ssp_time_end;
	static double x_move_amp, y_move_amp, z_move_amp, a_move_amp, b_move_amp, c_move_amp, z_vibe_amp;
	static double x_move_amp_shift, y_move_amp_shift, z_move_amp_shift, a_move_amp_shift, b_move_amp_shift, c_move_amp_shift, z_vibe_amp_shift;
	static double z_vibe_phase_shift;
	static double x_move, y_move, z_move, a_move, b_move, c_move, z_vibe;

	//	matd GtoRF;
	//	matd GtoLF;

	if(m_StepIdxData(ep_idx) == NO_STEP_IDX)
	{
		m_matvGtoLF[ep_idx] = GetTransformMatrix(m_ReferenceLeftFootPosition.x, m_ReferenceLeftFootPosition.y, m_ReferenceLeftFootPosition.z,
				m_ReferenceLeftFootPosition.roll, m_ReferenceLeftFootPosition.pitch, m_ReferenceLeftFootPosition.yaw);
		m_matvGtoRF[ep_idx] = GetTransformMatrix(m_ReferenceRightFootPosition.x, m_ReferenceRightFootPosition.y, m_ReferenceRightFootPosition.z,
				m_ReferenceRightFootPosition.roll, m_ReferenceRightFootPosition.pitch, m_ReferenceRightFootPosition.yaw);
		start_idx = 1;
	}
	else {
		if(step_idx != 0 )
			ref_time = m_StepData[step_idx-1].TimeData.dAbsStepTime;
		else
			ref_time = m_ReferenceTime;

		period_time = m_StepData[step_idx].TimeData.dAbsStepTime - ref_time;
		dsp_ratio = m_StepData[step_idx].TimeData.dDSPratio;
		ssp_ratio = 1 - dsp_ratio;
		foot_move_period_time = ssp_ratio*period_time;
		body_move_period_time = period_time;

		ssp_time_start = dsp_ratio*period_time/2.0;
		ssp_time_end = (1 + ssp_ratio)*period_time / 2.0;

		if(m_StepData[step_idx].PositionData.bMovingFoot == RFootMove)
		{
			x_move_amp = (m_StepData[step_idx].PositionData.stRightFootPosition.x - m_ReferenceRightFootPosition.x);
			x_move_amp_shift = m_ReferenceRightFootPosition.x;

			y_move_amp = (m_StepData[step_idx].PositionData.stRightFootPosition.y - m_ReferenceRightFootPosition.y);
			y_move_amp_shift = m_ReferenceRightFootPosition.y;

			z_move_amp = (m_StepData[step_idx].PositionData.stRightFootPosition.z - m_ReferenceRightFootPosition.z);
			z_move_amp_shift = m_ReferenceRightFootPosition.z;

			a_move_amp = (m_StepData[step_idx].PositionData.stRightFootPosition.roll - m_ReferenceRightFootPosition.roll);
			a_move_amp_shift = m_ReferenceRightFootPosition.roll;

			b_move_amp = (m_StepData[step_idx].PositionData.stRightFootPosition.pitch - m_ReferenceRightFootPosition.pitch);
			b_move_amp_shift = m_ReferenceRightFootPosition.pitch;

			c_move_amp = (m_StepData[step_idx].PositionData.stRightFootPosition.yaw - m_ReferenceRightFootPosition.yaw);
			c_move_amp_shift = m_ReferenceRightFootPosition.yaw;

			z_vibe_amp = m_StepData[step_idx].PositionData.dFootHeight*0.5;
			z_vibe_amp_shift = z_vibe_amp;
			z_vibe_phase_shift = PI*0.5;
		}
		else if(m_StepData[step_idx].PositionData.bMovingFoot == LFootMove)	{
			x_move_amp = (m_StepData[step_idx].PositionData.stLeftFootPosition.x - m_ReferenceLeftFootPosition.x);
			x_move_amp_shift = m_ReferenceLeftFootPosition.x;

			y_move_amp = (m_StepData[step_idx].PositionData.stLeftFootPosition.y - m_ReferenceLeftFootPosition.y);
			y_move_amp_shift = m_ReferenceLeftFootPosition.y;

			z_move_amp = (m_StepData[step_idx].PositionData.stLeftFootPosition.z - m_ReferenceLeftFootPosition.z);
			z_move_amp_shift = m_ReferenceLeftFootPosition.z;

			a_move_amp = (m_StepData[step_idx].PositionData.stLeftFootPosition.roll - m_ReferenceLeftFootPosition.roll);
			a_move_amp_shift = m_ReferenceLeftFootPosition.roll;

			b_move_amp = (m_StepData[step_idx].PositionData.stLeftFootPosition.pitch - m_ReferenceLeftFootPosition.pitch);
			b_move_amp_shift = m_ReferenceLeftFootPosition.pitch;

			c_move_amp = (m_StepData[step_idx].PositionData.stLeftFootPosition.yaw - m_ReferenceLeftFootPosition.yaw);
			c_move_amp_shift = m_ReferenceLeftFootPosition.yaw;

			z_vibe_amp = m_StepData[step_idx].PositionData.dFootHeight*0.5;
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
	}

	for(ep_idx = start_idx; ep_idx < m_DFilterSize; ep_idx++)
	{
		if(m_StepIdxData(ep_idx) == NO_STEP_IDX)
		{
			m_matvGtoLF[ep_idx] = m_matvGtoLF[ep_idx - 1];
			m_matvGtoRF[ep_idx] = m_matvGtoRF[ep_idx - 1];
		}
		else
		{
			if(step_idx != m_StepIdxData(ep_idx))
			{
				step_idx = m_StepIdxData(ep_idx);
				if(step_idx != 0)
					ref_time = m_StepData[step_idx-1].TimeData.dAbsStepTime;
				else
					ref_time = m_ReferenceTime;

				period_time = m_StepData[step_idx].TimeData.dAbsStepTime - ref_time;
				dsp_ratio = m_StepData[step_idx].TimeData.dDSPratio;
				ssp_ratio = 1 - dsp_ratio;
				foot_move_period_time = ssp_ratio*period_time;
				body_move_period_time = period_time;

				ssp_time_start = dsp_ratio*period_time/2.0;
				ssp_time_end = (1 + ssp_ratio)*period_time / 2.0;

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
			}


			time = m_WalkingTime + (ep_idx+1)*TIME_UNIT - ref_time;
			//			if(m_WalkingTime == 2992 || m_WalkingTime == 3000)
			//				printf("%d %d %d %d %f\n", step_idx, m_WalkingTime, ref_time, time, x_move_amp);

			if( time <= ssp_time_start)
			{
				x_move = wsigmoid(ssp_time_start, foot_move_period_time, ssp_time_start, x_move_amp, x_move_amp_shift, m_StepData[step_idx].TimeData.sigmoid_ratio_x,     m_StepData[step_idx].TimeData.sigmoid_distortion_x);
				y_move = wsigmoid(ssp_time_start, foot_move_period_time, ssp_time_start, y_move_amp, y_move_amp_shift, m_StepData[step_idx].TimeData.sigmoid_ratio_y,     m_StepData[step_idx].TimeData.sigmoid_distortion_y);
				z_move = wsigmoid(ssp_time_start, foot_move_period_time, ssp_time_start, z_move_amp, z_move_amp_shift, m_StepData[step_idx].TimeData.sigmoid_ratio_z,     m_StepData[step_idx].TimeData.sigmoid_distortion_z);
				a_move = wsigmoid(ssp_time_start, foot_move_period_time, ssp_time_start, a_move_amp, a_move_amp_shift, m_StepData[step_idx].TimeData.sigmoid_ratio_roll,  m_StepData[step_idx].TimeData.sigmoid_distortion_roll);
				b_move = wsigmoid(ssp_time_start, foot_move_period_time, ssp_time_start, b_move_amp, b_move_amp_shift, m_StepData[step_idx].TimeData.sigmoid_ratio_pitch, m_StepData[step_idx].TimeData.sigmoid_distortion_pitch);
				c_move = wsigmoid(ssp_time_start, foot_move_period_time, ssp_time_start, c_move_amp, c_move_amp_shift, m_StepData[step_idx].TimeData.sigmoid_ratio_yaw,   m_StepData[step_idx].TimeData.sigmoid_distortion_yaw);

				z_vibe = wsin(ssp_time_start, foot_move_period_time, z_vibe_phase_shift + 2.0*PI*ssp_time_start/foot_move_period_time, z_vibe_amp, z_vibe_amp_shift);
			}
			else if(time <= ssp_time_end) {
				x_move = wsigmoid(time, foot_move_period_time, ssp_time_start, x_move_amp, x_move_amp_shift, m_StepData[step_idx].TimeData.sigmoid_ratio_x,     m_StepData[step_idx].TimeData.sigmoid_distortion_x);
				y_move = wsigmoid(time, foot_move_period_time, ssp_time_start, y_move_amp, y_move_amp_shift, m_StepData[step_idx].TimeData.sigmoid_ratio_y,     m_StepData[step_idx].TimeData.sigmoid_distortion_y);
				z_move = wsigmoid(time, foot_move_period_time, ssp_time_start, z_move_amp, z_move_amp_shift, m_StepData[step_idx].TimeData.sigmoid_ratio_z,     m_StepData[step_idx].TimeData.sigmoid_distortion_z);
				a_move = wsigmoid(time, foot_move_period_time, ssp_time_start, a_move_amp, a_move_amp_shift, m_StepData[step_idx].TimeData.sigmoid_ratio_roll,  m_StepData[step_idx].TimeData.sigmoid_distortion_roll);
				b_move = wsigmoid(time, foot_move_period_time, ssp_time_start, b_move_amp, b_move_amp_shift, m_StepData[step_idx].TimeData.sigmoid_ratio_pitch, m_StepData[step_idx].TimeData.sigmoid_distortion_pitch);
				c_move = wsigmoid(time, foot_move_period_time, ssp_time_start, c_move_amp, c_move_amp_shift, m_StepData[step_idx].TimeData.sigmoid_ratio_yaw,   m_StepData[step_idx].TimeData.sigmoid_distortion_yaw);

				z_vibe = wsin(time, foot_move_period_time, z_vibe_phase_shift + 2.0*PI*ssp_time_start/foot_move_period_time, z_vibe_amp, z_vibe_amp_shift);
			}
			else {
				x_move = wsigmoid(ssp_time_end, foot_move_period_time, ssp_time_start, x_move_amp, x_move_amp_shift, m_StepData[step_idx].TimeData.sigmoid_ratio_x,     m_StepData[step_idx].TimeData.sigmoid_distortion_x);
				y_move = wsigmoid(ssp_time_end, foot_move_period_time, ssp_time_start, y_move_amp, y_move_amp_shift, m_StepData[step_idx].TimeData.sigmoid_ratio_y,     m_StepData[step_idx].TimeData.sigmoid_distortion_y);
				z_move = wsigmoid(ssp_time_end, foot_move_period_time, ssp_time_start, z_move_amp, z_move_amp_shift, m_StepData[step_idx].TimeData.sigmoid_ratio_z,     m_StepData[step_idx].TimeData.sigmoid_distortion_z);
				a_move = wsigmoid(ssp_time_end, foot_move_period_time, ssp_time_start, a_move_amp, a_move_amp_shift, m_StepData[step_idx].TimeData.sigmoid_ratio_roll,  m_StepData[step_idx].TimeData.sigmoid_distortion_roll);
				b_move = wsigmoid(ssp_time_end, foot_move_period_time, ssp_time_start, b_move_amp, b_move_amp_shift, m_StepData[step_idx].TimeData.sigmoid_ratio_pitch, m_StepData[step_idx].TimeData.sigmoid_distortion_pitch);
				c_move = wsigmoid(ssp_time_end, foot_move_period_time, ssp_time_start, c_move_amp, c_move_amp_shift, m_StepData[step_idx].TimeData.sigmoid_ratio_yaw,   m_StepData[step_idx].TimeData.sigmoid_distortion_yaw);

				z_vibe = wsin(ssp_time_end, foot_move_period_time, z_vibe_phase_shift + 2.0*PI*ssp_time_start/foot_move_period_time, z_vibe_amp, z_vibe_amp_shift);
			}



			if(m_StepData[step_idx].PositionData.bMovingFoot == RFootMove)
			{
				m_matvGtoRF[ep_idx] = GetTransformMatrix(x_move, y_move, z_move + z_vibe, a_move, b_move, c_move);
				m_matvGtoLF[ep_idx] = GetTransformMatrix(m_StepData[step_idx].PositionData.stLeftFootPosition.x, m_StepData[step_idx].PositionData.stLeftFootPosition.y, m_StepData[step_idx].PositionData.stLeftFootPosition.z,
						m_StepData[step_idx].PositionData.stLeftFootPosition.roll, m_StepData[step_idx].PositionData.stLeftFootPosition.pitch, m_StepData[step_idx].PositionData.stLeftFootPosition.yaw);
			}
			else if(m_StepData[step_idx].PositionData.bMovingFoot == LFootMove)	{
				m_matvGtoRF[ep_idx] = GetTransformMatrix(m_StepData[step_idx].PositionData.stRightFootPosition.x, m_StepData[step_idx].PositionData.stRightFootPosition.y, m_StepData[step_idx].PositionData.stRightFootPosition.z,
						m_StepData[step_idx].PositionData.stRightFootPosition.roll, m_StepData[step_idx].PositionData.stRightFootPosition.pitch, m_StepData[step_idx].PositionData.stRightFootPosition.yaw);
				m_matvGtoLF[ep_idx] = GetTransformMatrix(x_move, y_move, z_move + z_vibe, a_move, b_move, c_move);
			}
			else {

				m_matvGtoRF[ep_idx] = GetTransformMatrix(m_StepData[step_idx].PositionData.stRightFootPosition.x, m_StepData[step_idx].PositionData.stRightFootPosition.y, m_StepData[step_idx].PositionData.stRightFootPosition.z,
						m_StepData[step_idx].PositionData.stRightFootPosition.roll, m_StepData[step_idx].PositionData.stRightFootPosition.pitch, m_StepData[step_idx].PositionData.stRightFootPosition.yaw);
				m_matvGtoLF[ep_idx] = GetTransformMatrix(m_StepData[step_idx].PositionData.stLeftFootPosition.x, m_StepData[step_idx].PositionData.stLeftFootPosition.y, m_StepData[step_idx].PositionData.stLeftFootPosition.z,
						m_StepData[step_idx].PositionData.stLeftFootPosition.roll, m_StepData[step_idx].PositionData.stLeftFootPosition.pitch, m_StepData[step_idx].PositionData.stLeftFootPosition.yaw);
			}
		}
	}
}

void PreviewControlWalking::CalcCOBData(matd x_COB, matd y_COB)
{
	unsigned int cob_idx = 0;
	unsigned int start_idx = 0;
	int step_idx = m_StepIdxData(cob_idx);

	int time, ref_time = m_ReferenceTime;
	static double period_time, dsp_ratio, ssp_ratio, foot_move_period_time, body_move_period_time, ssp_time_start, ssp_time_end;
	static double z_swap_amp = 0, z_swap_amp_shift = 0;
	static double z_swap_phase_shift = PI*0.5;
	static double bz_move_amp = 0.0, bz_move_amp_shift = m_ReferenceBodyPosition.z;
	static double ba_move_amp = 0.0, ba_move_amp_shift = m_ReferenceBodyPosition.roll;
	static double bb_move_amp = 0.0, bb_move_amp_shift = m_ReferenceBodyPosition.pitch;
	static double bc_move_amp = 0.0, bc_move_amp_shift = m_ReferenceBodyPosition.yaw;
	static double z_swap, bz_move = 0.0, ba_move = 0.0, bb_move = 0.0, bc_move = 0.0;

	double cob_x, cob_y;

	if(m_StepIdxData(cob_idx) == NO_STEP_IDX)
	{
		cob_x = x_COB(0, cob_idx); cob_y = y_COB(0, cob_idx);
		m_matvGtoCOB[cob_idx] = GetTransformMatrix(cob_x, cob_y, m_ReferenceBodyPosition.z,
				m_ReferenceBodyPosition.roll, m_ReferenceBodyPosition.pitch, m_ReferenceBodyPosition.yaw);
		start_idx = 1;
	}
	else {
		step_idx = m_StepIdxData(cob_idx);
		period_time = m_StepData[step_idx].TimeData.dAbsStepTime - ref_time;
		dsp_ratio = m_StepData[step_idx].TimeData.dDSPratio;
		ssp_ratio = 1 - dsp_ratio;
		foot_move_period_time = ssp_ratio*period_time;
		body_move_period_time = period_time;

		ssp_time_start = dsp_ratio*period_time/2.0;
		ssp_time_end = (1 + ssp_ratio)*period_time / 2.0;

		z_swap_amp = m_StepData[step_idx].PositionData.dZ_Swap_Amplitude;
		z_swap_amp_shift = z_swap_amp;

		bz_move_amp = m_StepData[step_idx].PositionData.stBodyPosition.z - m_ReferenceBodyPosition.z;
		bz_move_amp_shift = m_ReferenceBodyPosition.z;

		ba_move_amp = m_StepData[step_idx].PositionData.stBodyPosition.roll - m_ReferenceBodyPosition.roll;
		ba_move_amp_shift = m_ReferenceBodyPosition.roll;

		bb_move_amp = m_StepData[step_idx].PositionData.stBodyPosition.pitch - m_ReferenceBodyPosition.pitch;
		bb_move_amp_shift = m_ReferenceBodyPosition.pitch;

		bc_move_amp = m_StepData[step_idx].PositionData.stBodyPosition.yaw - m_ReferenceBodyPosition.yaw;
		bc_move_amp_shift = m_ReferenceBodyPosition.yaw;
	}


	for(cob_idx = start_idx; cob_idx < m_DFilterSize; cob_idx++)
	{
		if(m_StepIdxData(cob_idx) == NO_STEP_IDX)
		{
			m_matvGtoLF[cob_idx] = m_matvGtoLF[cob_idx - 1];
			m_matvGtoRF[cob_idx] = m_matvGtoRF[cob_idx - 1];
		}
		else
		{
			if(step_idx != m_StepIdxData(cob_idx))
			{
				step_idx = m_StepIdxData(cob_idx);
				if(step_idx != 0)
					ref_time = m_StepData[step_idx-1].TimeData.dAbsStepTime;
				else
					ref_time = m_ReferenceTime;

				period_time = m_StepData[cob_idx].TimeData.dAbsStepTime - ref_time;
				dsp_ratio = m_StepData[cob_idx].TimeData.dDSPratio;
				ssp_ratio = 1 - dsp_ratio;
				foot_move_period_time = ssp_ratio*period_time;
				body_move_period_time = period_time;

				ssp_time_start = dsp_ratio*period_time/2.0;
				ssp_time_end = (1 + ssp_ratio)*period_time / 2.0;

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
			}

			time = m_WalkingTime + (cob_idx+1)*TIME_UNIT - ref_time;

			z_swap  = wsin(time, body_move_period_time, z_swap_phase_shift, z_swap_amp, z_swap_amp_shift);
			bz_move = wsigmoid(time, body_move_period_time, 0, bz_move_amp, bz_move_amp_shift, 1.0, 1.0);
			ba_move = wsigmoid(time, body_move_period_time, 0, ba_move_amp, ba_move_amp_shift, 1.0, 1.0);
			bb_move = wsigmoid(time, body_move_period_time, 0, bb_move_amp, bb_move_amp_shift, 1.0, 1.0);
			bc_move = wsigmoid(time, body_move_period_time, 0, bc_move_amp, bc_move_amp_shift, 1.0, 1.0);

			cob_x = x_COB(0, cob_idx); cob_y = y_COB(0, cob_idx);
			m_matvGtoCOB[cob_idx] = GetTransformMatrix(cob_x, cob_y, bz_move+z_swap, ba_move, bb_move, bc_move);
		}
	}
}

bool PreviewControlWalking::CalcZMPbyMPMM()
{
	static int idx = 0;
	static unsigned int zmp_idx = 0;
	int step_idx;
	static Pose3D epr, epl;
	static double angle[12] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
	static double rangle[6] = { 0, 0, 0, 0, 0, 0}, langle[6] = { 0, 0, 0, 0, 0, 0};
	static matd PData1_calc, PData2_calc, PData3_calc;
	static matd v1, v2, a;
	static double sum1 = 0.0, sum2 = 1.0;

	PData1_calc = PData2;
	PData2_calc = PData3;

	//0th Position is (k+1)th Position
	for(zmp_idx = 1; zmp_idx < m_DFilterSize; zmp_idx++)
	{
		matCOBtoG = GetTransformMatrixInverse(m_matvGtoCOB[zmp_idx-1]);
		matRHtoRF = matRHtoCOB*matCOBtoG*m_matvGtoRF[zmp_idx-1];
		matLHtoLF = matLHtoCOB*matCOBtoG*m_matvGtoLF[zmp_idx-1];

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


		step_idx = m_StepIdxData(zmp_idx-1);
		if(step_idx != -1)
		{
			r_arm[0] = (double)dir[12]*(epr.x - epl.x)*m_StepData[step_idx].PositionData.dShoulderSwingGain + r_arm_init[0];
			r_arm[3] = (double)dir[14]*(epr.x - epl.x)*m_StepData[step_idx].PositionData.dElbowSwingGain    + r_arm_init[3];

			l_arm[0] = (double)dir[13]*(epl.x - epr.x)*m_StepData[step_idx].PositionData.dShoulderSwingGain + l_arm_init[0];
			l_arm[3] = (double)dir[15]*(epl.x - epr.x)*m_StepData[step_idx].PositionData.dElbowSwingGain    + l_arm_init[3];
		}

		for(idx = 0; idx < 6; idx++)
		{
			rangle[idx] = angle[idx]*dir[idx]*180.0/PI;
			langle[idx] = angle[idx+6]*dir[idx+6]*180.0/PI;
		}

		PData3_calc = m_matvGtoCOB[zmp_idx-1]*GetCOMofMultiBody(langle, rangle, l_arm, r_arm);

		v1 = (PData2_calc - PData1_calc)/(TIME_UNIT*0.001);
		v2 = (PData3_calc - PData2_calc)/(TIME_UNIT*0.001);

		a = (v2 - v1)/(TIME_UNIT*0.001);

		sum1 = 0;  sum2 = 0;
		for(idx = 0; idx < 22; idx++)
		{
			sum1 += (Dynamics::Mass[idx])*(PData2_calc(0, idx)*(a(2, idx) + G) - a(0, idx)*PData2_calc(2, idx));
			sum2 += (Dynamics::Mass[idx])*(a(2, idx) + G);
			//sum2 += Dynamics::Mass[idx]*(PData2_calc(1, idx)*(a(2, idx) + G) - a(1, idx)*PData2_calc(2, idx));
		}

		m_ZMP_Calculated_by_MPMM_X_post(zmp_idx-1, 0) = sum1/sum2;

		sum1 = 0;
		for(idx = 0; idx < 22; idx++)
		{
			sum1 += Dynamics::Mass[idx]*(PData2_calc(1, idx)*(a(2, idx) + G) - a(1, idx)*PData2_calc(2, idx));
		}

		m_ZMP_Calculated_by_MPMM_Y_post(zmp_idx-1, 0) = sum1/sum2;

		PData1_calc = PData2_calc;
		PData2_calc = PData3_calc;
	}

	m_ZMP_Calculated_by_MPMM_X_post(m_DFilterSize-1, 0) = m_ZMP_Calculated_by_MPMM_X_post(m_DFilterSize-2, 0);
	m_ZMP_Calculated_by_MPMM_Y_post(m_DFilterSize-1, 0) = m_ZMP_Calculated_by_MPMM_Y_post(m_DFilterSize-2, 0);

	return true;
}

void PreviewControlWalking::CalcDHforRightArm(double *rightArmAngle_deg, matd *T01, matd *T02, matd *T03, matd *T04, matd *T05, matd *T06)
{
	matd t1, t2, t3, t4, t5, t6;
	t1 = CalcDH(     0,  PI/2.0,     0, rightArmAngle_deg[0]*PI/180.0);
	t2 = CalcDH(     0, -PI/2.0,     0, rightArmAngle_deg[1]*PI/180.0);
	t3 = CalcDH(  30.0, -PI/2.0, 246.0, rightArmAngle_deg[2]*PI/180.0);
	t4 = CalcDH( -30.0,  PI/2.0,     0, rightArmAngle_deg[3]*PI/180.0);
	t5 = CalcDH(     0, -PI/2.0, 135.0, rightArmAngle_deg[4]*PI/180.0);
	t6 = CalcDH(     0,       0,     0, rightArmAngle_deg[5]*PI/180.0);

	*T01 = t1;
	*T02 = (*T01)*t2;
	*T03 = (*T02)*t3;
	*T04 = (*T03)*t4;
	*T05 = (*T04)*t5;
	*T06 = (*T05)*t6;
}

void PreviewControlWalking::CalcDHforLeftArm(double *leftArmAngle_deg, matd *T01, matd *T02, matd *T03, matd *T04, matd *T05, matd *T06)
{
	matd t1, t2, t3, t4, t5, t6;
	t1 = CalcDH(     0, -PI/2.0,     0, leftArmAngle_deg[0]*PI/180.0);
	t2 = CalcDH(     0,  PI/2.0,     0, leftArmAngle_deg[1]*PI/180.0);
	t3 = CalcDH(  30.0,  PI/2.0, 246.0, leftArmAngle_deg[2]*PI/180.0);
	t4 = CalcDH( -30.0, -PI/2.0,     0, leftArmAngle_deg[3]*PI/180.0);
	t5 = CalcDH(     0,  PI/2.0, 135.0, leftArmAngle_deg[4]*PI/180.0);
	t6 = CalcDH(     0,       0,     0, leftArmAngle_deg[5]*PI/180.0);

	*T01 = t1;
	*T02 = (*T01)*t2;
	*T03 = (*T02)*t3;
	*T04 = (*T03)*t4;
	*T05 = (*T04)*t5;
	*T06 = (*T05)*t6;
}

void PreviewControlWalking::CalcDHforRightLeg(double *rightLegAngle_deg, matd *T01, matd *T02, matd *T03, matd *T04, matd *T05, matd *T06)
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

void PreviewControlWalking::CalcDHforLeftLeg(double *leftLegAngle_deg, matd *T01, matd *T02, matd *T03, matd *T04, matd *T05, matd *T06)
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

matd PreviewControlWalking::GetCOMofMultiBody(double* leftLegAngle_deg, double* rightLegAngle_deg, double* leftArmAngle_deg, double* rightArmAngle_deg)
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

	vCOM(0) =  0.0; vCOM(1) = 46.4; vCOM(2) = 0.0;
	COM_Pos_List.col(14) = matCOBtoRSholder*T05*vCOM;

	vCOM(0) = 3.9; vCOM(1) = 146.0; vCOM(2) = 0.0;
	COM_Pos_List.col(15) = matCOBtoRSholder*T06*vCOM;

	//16~19 LeftArm
	matd matCOBtoLShoulder = GetTranslationMatrix(0, 234, 447)*GetOrientationMatrix(-PI*0.5, PI*0.5, 0.0);
	CalcDHforLeftArm(leftArmAngle_deg, &T01, &T02, &T03, &T04, &T05, &T06);
	vCOM(0) = -22.0; vCOM(1) = -143.3; vCOM(2) = 0.0;
	COM_Pos_List.col(16) = matCOBtoLShoulder*T03*vCOM;

	vCOM(0) =  0.0; vCOM(1) = 0.0; vCOM(2) = 35.1;
	COM_Pos_List.col(17) = matCOBtoLShoulder*T04*vCOM;

	vCOM(0) =  0.0; vCOM(1) = -46.4; vCOM(2) = 0.0;
	COM_Pos_List.col(18) = matCOBtoLShoulder*T05*vCOM;

	vCOM(0) = 3.9; vCOM(1) = 146.0; vCOM(2) = 0.0;
	COM_Pos_List.col(19) = matCOBtoLShoulder*T06*vCOM;


	//20 body_lower
	vCOM(0) = -26.4; vCOM(1) = 0; vCOM(2) = 161.2;
	COM_Pos_List.col(20) = vCOM;

	//21 body_upper
	vCOM(0) = -20.8; vCOM(1) = 0; vCOM(2) = 155.7;
	COM_Pos_List.col(21) = GetTransformMatrix(0, 0, 282, 0, 0, 0)*vCOM;

	return COM_Pos_List;
}

void PreviewControlWalking::Process()
{
	if(!m_Ctrl_Running && !m_Real_Running)
		return;

	while(step_data_locker);

	step_data_locker = true;

	if(m_StepData.size() == 0)
		return;

	static double angle[16];
	static int outValue[16];
	static double rangle[6], langle[6];
	static int idx = 0;


	static Pose3D epr, epl;


	CalcStepIdxData();
	CalcRefZMP();

	CalcEndPointData();

	int k = 0;

	u_x = -K*x_LIPM.col(k) + f_Preview*m_ZMP_Reference_X.block(k,0,m_PreviewSize,1);
	x_LIPM.col(k) = A*x_LIPM.col(k) + b*u_x;
	m_ZMP_Generated_by_LIPM_X(k, 0) = (c*x_LIPM.col(k))(0, 0);

	u_y = -K*y_LIPM.col(k) + f_Preview*m_ZMP_Reference_Y.block(k,0,m_PreviewSize,1);
	y_LIPM.col(k) = A*y_LIPM.col(k) + b*u_y;
	m_ZMP_Generated_by_LIPM_Y(k, 0) = (c*y_LIPM.col(k))(0, 0);

	for(k = 0; k < m_DFilterSize-1; k++)
	{
		u_x = -K*x_LIPM.col(k) + f_Preview*m_ZMP_Reference_X.block(k,0,m_PreviewSize,1);
		x_LIPM.col(k+1) = A*x_LIPM.col(k) + b*u_x;
		m_ZMP_Generated_by_LIPM_X(k, 0) = (c*x_LIPM.col(k))(0, 0);

		u_y = -K*y_LIPM.col(k) + f_Preview*m_ZMP_Reference_Y.block(k,0,m_PreviewSize,1);
		y_LIPM.col(k+1) = A*y_LIPM.col(k) + b*u_y;
		m_ZMP_Generated_by_LIPM_Y(k, 0) = (c*y_LIPM.col(k))(0, 0);
	}

	m_ZMP_Generated_by_LIPM_X(m_DFilterSize-1, 0) = (c*x_LIPM.col(m_DFilterSize-1))(0, 0);
	m_ZMP_Generated_by_LIPM_Y(m_DFilterSize-1, 0) = (c*y_LIPM.col(m_DFilterSize-1))(0, 0);


	printf("%d %d %d\n", m_StepIdxData(0), m_WalkingTime, m_ReferenceTime);

	k = 0;
	if(FILTERING_ENABLE)
	{
		CalcCOBData(x_LIPM, y_LIPM);
		CalcZMPbyMPMM();

		u_delta_x = -K*x_delta.col(k) + f_Dfilter*(m_ZMP_Generated_by_LIPM_X - m_ZMP_Calculated_by_MPMM_X_post);
		x_delta.col(k) = A*x_delta.col(k) + b*u_delta_x;
		x_MPMM_pre.col(k) = x_LIPM.col(k) + x_delta.col(k);

		u_delta_y = -K*y_delta.col(k) + f_Dfilter*(m_ZMP_Generated_by_LIPM_Y - m_ZMP_Calculated_by_MPMM_Y_post);
		y_delta.col(k) = A*y_delta.col(k) + b*u_delta_y;
		y_MPMM_pre.col(k) = y_LIPM.col(k) + y_delta.col(k);
	}
	else
	{
		x_MPMM_pre.col(k) = x_LIPM.col(k);
		y_MPMM_pre.col(k) = y_LIPM.col(k);
	}
	PData1 = PData2;
	PData2 = PData3;

	//printf("%d %d %f %f %f %f %f\n", m_WalkingTime, m_StepIdxData(0), m_ZMP_Reference_X(0), x_LIPM(0,0), m_ZMP_Calculated_by_MPMM_X_post(0), m_matvGtoLF[0](0,3), m_matvGtoRF[0](0,3));
	//printf("%d %d %f %f %f %f\n", m_WalkingTime, m_StepIdxData(0), m_ZMP_Reference_Y(0), y_LIPM(0,0), m_matvGtoLF[0](1,3), m_matvGtoRF[0](1,3));
	//printf("%d %f %f %f %f\n", m_StepIdxData(0), x_LIPM(0,0), m_ZMP_Reference_X(0), m_ZMP_Generated_by_LIPM_X(0), m_ZMP_Calculated_by_MPMM_X_post(0));
	//printf("%d %f %f %f %f %f\n", m_StepIdxData(0), x_LIPM(0,0), x_MPMM_pre(0,0), m_ZMP_Reference_X(0), m_ZMP_Generated_by_LIPM_X(0), m_ZMP_Calculated_by_MPMM_X_post(0));

	m_Body_Move_PeriodTime = m_StepData[m_PresentStepNum].TimeData.dAbsStepTime - m_ReferenceTime;
	m_DSP_Ratio = m_StepData[m_PresentStepNum].TimeData.dDSPratio;
	m_SSP_Ratio = 1 - m_DSP_Ratio;
	//m_Foot_Move_PeriodTime = m_SSP_Ratio*m_Body_Move_PeriodTime;
	m_SSP_Time_Start = m_DSP_Ratio*m_Body_Move_PeriodTime/2.0;
	m_SSP_Time_End = (1 + m_SSP_Ratio)*m_Body_Move_PeriodTime / 2.0;

	double bz_move_amp = m_StepData[m_PresentStepNum].PositionData.stBodyPosition.z - m_ReferenceBodyPosition.z;
	double bz_move_amp_shift = m_ReferenceBodyPosition.z;

	double ba_move_amp = m_StepData[m_PresentStepNum].PositionData.stBodyPosition.roll - m_ReferenceBodyPosition.roll;
	double ba_move_amp_shift = m_ReferenceBodyPosition.roll;

	double bb_move_amp = m_StepData[m_PresentStepNum].PositionData.stBodyPosition.pitch - m_ReferenceBodyPosition.pitch;
	double bb_move_amp_shift = m_ReferenceBodyPosition.pitch;

	double bc_move_amp = m_StepData[m_PresentStepNum].PositionData.stBodyPosition.yaw - m_ReferenceBodyPosition.yaw;
	double bc_move_amp_shift = m_ReferenceBodyPosition.yaw;

	double z_swap_amp = m_StepData[m_PresentStepNum].PositionData.dZ_Swap_Amplitude;
	double z_swap_amp_shift = z_swap_amp;
	double z_swap_phase_shift = PI*0.5;

	double z_swap  = wsin(m_WalkingTime - m_ReferenceTime, m_Body_Move_PeriodTime, z_swap_phase_shift, z_swap_amp, z_swap_amp_shift);
	double bz_move = wsigmoid(m_WalkingTime - m_ReferenceTime, m_Body_Move_PeriodTime, 0, bz_move_amp, bz_move_amp_shift, 1.0, 1.0);
	double ba_move = wsigmoid(m_WalkingTime - m_ReferenceTime, m_Body_Move_PeriodTime, 0, ba_move_amp, ba_move_amp_shift, 1.0, 1.0);
	double bb_move = wsigmoid(m_WalkingTime - m_ReferenceTime, m_Body_Move_PeriodTime, 0, bb_move_amp, bb_move_amp_shift, 1.0, 1.0);
	double bc_move = wsigmoid(m_WalkingTime - m_ReferenceTime, m_Body_Move_PeriodTime, 0, bc_move_amp, bc_move_amp_shift, 1.0, 1.0);

	//////////////////// for Calc
	double cob_x = x_LIPM(0,0), cob_y = y_LIPM(0,0);
	matGtoCOB = GetTransformMatrix(cob_x, cob_y, z_swap + bz_move, ba_move, bb_move, bc_move);
	matCOBtoG = GetTransformMatrixInverse(matGtoCOB);
	matRHtoRF = matRHtoCOB*matCOBtoG*m_matvGtoRF[0];
	matLHtoLF = matLHtoCOB*matCOBtoG*m_matvGtoLF[0];

	epr = GetPose3DfromTransformMatrix(matRHtoRF);
	epl = GetPose3DfromTransformMatrix(matLHtoLF);

	if(computeIK(&angle[0], epr.x, epr.y, epr.z + Kinematics::LEG_LENGTH, epr.roll, epr.pitch, epr.yaw) == false)
	{
		printf("IK not Solved\n");
		return;
	}

	if(computeIK(&angle[6], epl.x, epl.y, epl.z + Kinematics::LEG_LENGTH, epl.roll, epl.pitch, epl.yaw) == false)
	{
		printf("IK not Solved\n");
		return;
	}

	for(idx = 0; idx < 6; idx++)
	{
		rangle[idx] = (double)dir[idx]*angle[idx]*180.0/PI;
		langle[idx] = (double)dir[idx+6]*angle[idx+6]*180.0/PI;
	}

	//	for(int idx = 0; idx < 6; idx++)
	//	{
	//		angle[idx] = (double)dir_output[idx]*angle[idx]*180.0/PI + InitAngle[idx];
	//		angle[idx+6] = (double)dir_output[idx+6]*angle[idx+6]*180.0/PI + InitAngle[idx+6];
	//	}
	//
	//	angle[12] = (double)dir_output[12]*(epr.x - epl.x)*m_StepData[m_PresentStepNum].PositionData.dShoulderSwingGain + InitAngle[12];
	//	angle[13] = (double)dir_output[13]*(epl.x - epr.x)*m_StepData[m_PresentStepNum].PositionData.dShoulderSwingGain + InitAngle[13];
	//	angle[14] = (double)dir_output[14]*(epr.x - epl.x)*m_StepData[m_PresentStepNum].PositionData.dElbowSwingGain    + InitAngle[14];
	//	angle[15] = (double)dir_output[15]*(epl.x - epr.x)*m_StepData[m_PresentStepNum].PositionData.dElbowSwingGain    + InitAngle[15];

	r_arm[0] = (double)dir[12]*(epr.x - epl.x)*m_StepData[m_PresentStepNum].PositionData.dShoulderSwingGain + r_arm_init[0];
	r_arm[3] = (double)dir[14]*(epr.x - epl.x)*m_StepData[m_PresentStepNum].PositionData.dElbowSwingGain    + r_arm_init[3];

	l_arm[0] = (double)dir[13]*(epl.x - epr.x)*m_StepData[m_PresentStepNum].PositionData.dShoulderSwingGain + l_arm_init[0];
	l_arm[3] = (double)dir[15]*(epl.x - epr.x)*m_StepData[m_PresentStepNum].PositionData.dElbowSwingGain    + l_arm_init[3];

	PData3 = matGtoCOB*GetCOMofMultiBody(langle, rangle, l_arm, r_arm);

	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	/////////////// for Command ans Debug
	cob_x = x_MPMM_pre(0,0), cob_y = y_MPMM_pre(0,0);
	//double cob_x = x_MPMM_pre(0,0), cob_y = y_MPMM_pre(0,0);
	//double cob_x = x_LIPM(0,0), cob_y = y_LIPM(0,0);
	matGtoCOB = GetTransformMatrix(cob_x, cob_y, z_swap + bz_move, ba_move, bb_move, bc_move);
	matCOBtoG = GetTransformMatrixInverse(matGtoCOB);
	matRHtoRF = matRHtoCOB*matCOBtoG*m_matvGtoRF[0];
	matLHtoLF = matLHtoCOB*matCOBtoG*m_matvGtoLF[0];

	epr = GetPose3DfromTransformMatrix(matRHtoRF);
	epl = GetPose3DfromTransformMatrix(matLHtoLF);



	// for Balancing Code
	// declaration of Balancing_Idx
	if(BALANCE_ENABLE) {
		if(m_StepData[m_PresentStepNum].TimeData.bWalkingState == InWalking)
		{
			if(m_WalkingTime - m_ReferenceTime < m_SSP_Time_Start)
			{
				if(m_StepData[m_PresentStepNum].PositionData.bMovingFoot == RFootMove)
				{
					m_Balancing_Index = 1;
				}
				else if(m_StepData[m_PresentStepNum].PositionData.bMovingFoot == LFootMove) {
					m_Balancing_Index = 5;
				}
				else {
					//ildan go.
					m_Balancing_Index = 0;
				}
			}
			else if(m_WalkingTime - m_ReferenceTime < (m_SSP_Time_Start+m_SSP_Time_End)*0.5) {
				if(m_StepData[m_PresentStepNum].PositionData.bMovingFoot == RFootMove)
				{
					m_Balancing_Index = 2;
				}
				else if(m_StepData[m_PresentStepNum].PositionData.bMovingFoot == LFootMove) {
					m_Balancing_Index = 6;
				}
				else {
					m_Balancing_Index = 0;
				}
			}
			else if(m_WalkingTime - m_ReferenceTime < m_SSP_Time_End) {
				if(m_StepData[m_PresentStepNum].PositionData.bMovingFoot == RFootMove)
				{
					m_Balancing_Index = 3;
				}
				else if(m_StepData[m_PresentStepNum].PositionData.bMovingFoot == LFootMove) {
					m_Balancing_Index = 7;
				}
				else {
					m_Balancing_Index = 0;
				}
			}
			else {
				if(m_StepData[m_PresentStepNum].PositionData.bMovingFoot == RFootMove)
				{
					m_Balancing_Index = 4;
				}
				else if(m_StepData[m_PresentStepNum].PositionData.bMovingFoot == LFootMove) {
					m_Balancing_Index = 8;
				}
				else {
					m_Balancing_Index = 0;
				}
			}
		}
		else if(m_StepData[m_PresentStepNum].TimeData.bWalkingState == InWalkingStarting)
			m_Balancing_Index = 0;
		else if(m_StepData[m_PresentStepNum].TimeData.bWalkingState == InWalkingEnding)
			m_Balancing_Index = 9;


		//Balancing Algorithm
		double ankleLength = Kinematics::ANKLE_LENGTH;

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

	if(computeIK(&angle[0], epr.x, epr.y, epr.z + Kinematics::LEG_LENGTH, epr.roll, epr.pitch, epr.yaw) == false)	{
		printf("IK not Solved\n");
		return;
	}

	if(computeIK(&angle[6], epl.x, epl.y, epl.z + Kinematics::LEG_LENGTH, epl.roll, epl.pitch, epl.yaw) == false)	{
		printf("IK not Solved\n");
		return;
	}

	for(idx = 0; idx < 6; idx++)
	{
		rangle[idx] = (double)dir[idx]*angle[idx]*180.0/PI;
		langle[idx] = (double)dir[idx+6]*angle[idx+6]*180.0/PI;
	}

	for(idx = 0; idx < 6; idx++)
	{
		angle[idx] = (double)dir_output[idx]*angle[idx]*180.0/PI + InitAngle[idx];
		angle[idx+6] = (double)dir_output[idx+6]*angle[idx+6]*180.0/PI + InitAngle[idx+6];
	}

	angle[12] = (double)dir_output[12]*(epr.x - epl.x)*m_StepData[m_PresentStepNum].PositionData.dShoulderSwingGain + InitAngle[12];
	angle[13] = (double)dir_output[13]*(epl.x - epr.x)*m_StepData[m_PresentStepNum].PositionData.dShoulderSwingGain + InitAngle[13];
	angle[14] = (double)dir_output[14]*(epr.x - epl.x)*m_StepData[m_PresentStepNum].PositionData.dElbowSwingGain    + InitAngle[14];
	angle[15] = (double)dir_output[15]*(epl.x - epr.x)*m_StepData[m_PresentStepNum].PositionData.dElbowSwingGain    + InitAngle[15];


	r_arm[0] = (double)dir[12]*(epr.x - epl.x)*m_StepData[m_PresentStepNum].PositionData.dShoulderSwingGain + r_arm_init[0];
	r_arm[3] = (double)dir[14]*(epr.x - epl.x)*m_StepData[m_PresentStepNum].PositionData.dElbowSwingGain    + r_arm_init[3];

	l_arm[0] = (double)dir[13]*(epl.x - epr.x)*m_StepData[m_PresentStepNum].PositionData.dShoulderSwingGain + l_arm_init[0];
	l_arm[3] = (double)dir[15]*(epl.x - epr.x)*m_StepData[m_PresentStepNum].PositionData.dElbowSwingGain    + l_arm_init[3];


	FilterdPData3 = GetCOMofMultiBody(langle, rangle, l_arm, r_arm);
	FilterdPData3 = matGtoCOB*FilterdPData3;


	if(DEBUG_PRINT)
	{
		matd v1 = (FilterdPData2 - FilterdPData1)/(TIME_UNIT*0.001);
		matd v2 = (FilterdPData3 - FilterdPData2)/(TIME_UNIT*0.001);

		matd a = (v2 - v1)/(TIME_UNIT*0.001);

		double sum1 = 0,  sum2 = 1;
		for(int idx = 0; idx < 22; idx++)
		{
			sum1 += (Dynamics::Mass[idx])*(FilterdPData2(0, idx)*(a(2, idx) + G) - a(0, idx)*FilterdPData2(2, idx));
			sum2 += (Dynamics::Mass[idx])*(a(2, idx) + G);
			//sum2 += Dynamics::Mass[idx]*(PData2_calc(1, idx)*(a(2, idx) + G) - a(1, idx)*PData2_calc(2, idx));
		}

		double m_ZMP_Generated_by_MPMM_X = sum1/sum2;

		sum1 = 0;
		for(int idx = 0; idx < 22; idx++)
		{
			sum1 += Dynamics::Mass[idx]*(FilterdPData2(1, idx)*(a(2, idx) + G) - a(1, idx)*FilterdPData2(2, idx));
		}

		double m_ZMP_Generated_by_MPMM_Y = sum1/sum2;
	}

	FilterdPData1 = FilterdPData2;
	FilterdPData2 = FilterdPData3;



	m_WalkingTime += TIME_UNIT;
	if(m_WalkingTime >= m_StepData[m_PresentStepNum].TimeData.dAbsStepTime)
	{
		m_ReferenceTime = m_StepData[m_PresentStepNum].TimeData.dAbsStepTime;
		m_WalkingTime = m_StepData[m_PresentStepNum].TimeData.dAbsStepTime;

		m_ReferenceRightFootPosition.x  = m_StepData[m_PresentStepNum].PositionData.stRightFootPosition.x;
		m_ReferenceRightFootPosition.y  = m_StepData[m_PresentStepNum].PositionData.stRightFootPosition.y;
		m_ReferenceRightFootPosition.z  = m_StepData[m_PresentStepNum].PositionData.stRightFootPosition.z;
		m_ReferenceRightFootPosition.roll  = m_StepData[m_PresentStepNum].PositionData.stRightFootPosition.roll;
		m_ReferenceRightFootPosition.pitch  = m_StepData[m_PresentStepNum].PositionData.stRightFootPosition.pitch;
		m_ReferenceRightFootPosition.yaw  = m_StepData[m_PresentStepNum].PositionData.stRightFootPosition.yaw;


		m_ReferenceLeftFootPosition.x  = m_StepData[m_PresentStepNum].PositionData.stLeftFootPosition.x;
		m_ReferenceLeftFootPosition.y  = m_StepData[m_PresentStepNum].PositionData.stLeftFootPosition.y;
		m_ReferenceLeftFootPosition.z  = m_StepData[m_PresentStepNum].PositionData.stLeftFootPosition.z;
		m_ReferenceLeftFootPosition.roll  = m_StepData[m_PresentStepNum].PositionData.stLeftFootPosition.roll;
		m_ReferenceLeftFootPosition.pitch  = m_StepData[m_PresentStepNum].PositionData.stLeftFootPosition.pitch;
		m_ReferenceLeftFootPosition.yaw  = m_StepData[m_PresentStepNum].PositionData.stLeftFootPosition.yaw;


		m_ReferenceBodyPosition.z  = m_StepData[m_PresentStepNum].PositionData.stBodyPosition.z;
		m_ReferenceBodyPosition.roll  = m_StepData[m_PresentStepNum].PositionData.stBodyPosition.roll;
		m_ReferenceBodyPosition.pitch  = m_StepData[m_PresentStepNum].PositionData.stBodyPosition.pitch;
		m_ReferenceBodyPosition.yaw  = m_StepData[m_PresentStepNum].PositionData.stBodyPosition.yaw;

		m_PresentStepNum += 1;
	}

	if(m_WalkingTime >= m_StepData[m_StepData.size()-1].TimeData.dAbsStepTime)
	{
		m_WalkingTime = m_StepData[m_StepData.size()-1].TimeData.dAbsStepTime;
		m_PresentStepNum -= 1;
		m_ReferenceTime = m_StepData[m_PresentStepNum -1].TimeData.dAbsStepTime;
	}

	step_data_locker = false;

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

bool PreviewControlWalking::computeIK(double *out, double x, double y, double z, double a, double b, double c)
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

//
//void PreviewControlWalking::CheckPresentStepNum()
//{
//	for(unsigned int step_data_idx = m_PresentStepNum; step_data_idx < m_StepData.size(); step_data_idx++)
//		if(m_WalkingTime <= m_StepData[step_data_idx])
//		{
//			m_PresentStepNum = step_data_idx;
//			break;
//			if(m_PresentStepNum != 0)
//				m_PreviousStepNum = m_PresentStepNum - 1;
//			else
//				m_PreviousStepNum = 0;
//		}
//}

////void PreviewControlWalking::RefreshStepData()
////{
////	CheckPresentStepNum();
////	m_StepData.erase(m_StepData.begin(), m_StepData.begin() + m_PresentStepNum);
////	m_PresentStepNum = 0;
////}
//
//bool PreviewControlWalking::GetRefZMP(int time, matd* pRefXZMP, matd* pRefYZMP)
//{
//	unsigned int step_idx;
//	unsigned int step_data_size = m_StepData.size();
////	if( (pRefXZMP->size() !=  (m_PreviewSize + m_DFilterSize)) || (pRefYZMP->size() != (m_PreviewSize + m_DFilterSize)) ) // �좎떛怨ㅼ삕 �좎룞���좎룞�쇿뜝�섎룄 �좎떖�몄삕?
////	{
////		pRefXZMP->resize(m_PreviewSize + m_DFilterSize);
////		pRefYZMP->resize(m_PreviewSize + m_DFilterSize);
////		//return false;
////	}
//
//	if(step_data_size == 0) // �좎룞�쇿뜝�숈삕 �좎룞�쇿뜝�숈삕�좑옙�좎룞�쇿뜝�숈삕�좑옙�좎뙐琉꾩삕 �좎룞�쇿뜝占�/		return false;
//	else {
//		//ref_zmp_idx�좎룞���좎떬諭꾩삕�좎룞���좎룞�쇿뜝�숈삕�좑옙�좎룞�쇿뜝�숈삕�좎떬�먯삕. Previewsize + Dfilrter Size�좎룞��//		for(int ref_zmp_idx = 0;  ref_zmp_idx < m_PreviewSize + m_DFilterSize; ref_zmp_idx ++){
//
//			if(time + (ref_zmp_idx +1)*TIME_UNIT <= m_StepData[step_data_size-1].TimeData.dAbsStepTime){
//				//step_idx�좎룞��李얍뜝�밸뙋��
//				for(step_idx = 0; step_idx < step_data_size; step_idx++) {
//					if(time + (ref_zmp_idx + 1)*TIME_UNIT <= m_StepData[step_idx].TimeData.dAbsStepTime)
//						break;
//				}//for end
//
//				if(m_StepData[step_idx].TimeData.bWalkingState == InWalking) {
//					if(m_StepData[step_idx].PositionData.bMovingFoot == LFootMove) {
//						//When  left foot move, the ref. zmp is set avrg. of left foot position
//						pRefXZMP->coeffRef(ref_zmp_idx, 0) = m_StepData[step_idx].PositionData.stRightFootPosition.x + m_X_ZMP_CenterShift;
//						pRefYZMP->coeffRef(ref_zmp_idx, 0) = m_StepData[step_idx].PositionData.stRightFootPosition.y + m_Y_ZMP_CenterShift + m_Y_ZMP_Convergence;
//					}
//					else if(m_StepData[step_idx].PositionData.bMovingFoot == RFootMove) {
//						//When  right foot move, the ref. zmp is set avrg. of left foot position
//						pRefXZMP->coeffRef(ref_zmp_idx, 0) = m_StepData[step_idx].PositionData.stLeftFootPosition.x + m_X_ZMP_CenterShift;
//						pRefYZMP->coeffRef(ref_zmp_idx, 0) = m_StepData[step_idx].PositionData.stLeftFootPosition.y + m_Y_ZMP_CenterShift - m_Y_ZMP_Convergence;
//					}
//					else if(m_StepData[step_idx].PositionData.bMovingFoot == NFootMove) {
//						//When Foot dose not move, the ref. zmp is set avrg. of two foot position
//						pRefXZMP->coeffRef(ref_zmp_idx, 0) = (m_StepData[step_idx].PositionData.stRightFootPosition.x + m_StepData[step_idx].PositionData.stLeftFootPosition.x)*0.5;
//						pRefYZMP->coeffRef(ref_zmp_idx, 0) = (m_StepData[step_idx].PositionData.stRightFootPosition.y + m_StepData[step_idx].PositionData.stLeftFootPosition.y)*0.5;
//					}
//					else
//						return false;
//				}
//				else if(m_StepData[step_idx].TimeData.bWalkingState == InWalkingStarting) {
//					pRefXZMP->coeffRef(ref_zmp_idx, 0) = (m_StepData[step_data_size-1].PositionData.stRightFootPosition.x + m_StepData[step_data_size-1].PositionData.stLeftFootPosition.x)*0.5 + m_X_ZMP_Init;
//					pRefYZMP->coeffRef(ref_zmp_idx, 0) = (m_StepData[step_data_size-1].PositionData.stRightFootPosition.y + m_StepData[step_data_size-1].PositionData.stLeftFootPosition.y)*0.5;
//				}
//				else if(m_StepData[step_idx].TimeData.bWalkingState == InWalkingEnding) {
//					pRefXZMP->coeffRef(ref_zmp_idx, 0) = (m_StepData[step_data_size-1].PositionData.stRightFootPosition.x + m_StepData[step_data_size-1].PositionData.stLeftFootPosition.x)*0.5 + m_X_ZMP_Init;
//					pRefYZMP->coeffRef(ref_zmp_idx, 0) = (m_StepData[step_data_size-1].PositionData.stRightFootPosition.y + m_StepData[step_data_size-1].PositionData.stLeftFootPosition.y)*0.5;
//				}
//			}
//			else {
//				pRefXZMP->coeffRef(ref_zmp_idx, 0) = (m_StepData[step_data_size-1].PositionData.stRightFootPosition.x + m_StepData[step_data_size-1].PositionData.stLeftFootPosition.x)*0.5;
//				pRefYZMP->coeffRef(ref_zmp_idx, 0) = (m_StepData[step_data_size-1].PositionData.stRightFootPosition.y + m_StepData[step_data_size-1].PositionData.stLeftFootPosition.y)*0.5;
//			}
//		}//for end
//
//	}//else end
//
//	return true;
//}
//
//bool PreviewControlWalking::GetEndPointData(double cob_x, double cob_y, int time,  matd* pmatGtoCOB, matd* pmatRHtoRF, matd* pmatLHtoLF)
//{
//	unsigned int step_idx;
//	unsigned int step_data_size = m_StepData.size();
//	if(step_data_size == 0) // �좎룞�쇿뜝�숈삕 �좎룞�쇿뜝�숈삕�좑옙�좎룞�쇿뜝�숈삕�좑옙�좎뙐琉꾩삕 �좎룞�쇿뜝占�/		return false;
//
//
//	return true;
//}
//
//void PreviewControlWalking::Process()
//{
//	static double angle[16];
//	static int outValue[16];
//
//	static double u_x, u_y;
//	static double u_delta_x, u_delta_y;
//
//	static matd _temp1, _temp2, _temp3;
//
//	//RefreshStepData();
//	CheckPresentStepNum();
//
//
//	m_Time = m_WalkingTime - m_ReferenceTime;
//	for(int k = 0; k < m_DFilterSize; k++)
//	{
////		if( GetRefZMP(m_WalkingTime + (k+1)*TIME_UNIT, &m_ZMP_Reference_X, &m_ZMP_Reference_Y) == false )
////			return;
//
//		u_x = -K*x_LIPM.col(k) + f_Preview*m_ZMP_Reference_X;
//		x_LIPM.col(k+1) = A*x_LIPM.col(k) + b*u_x;
//
//		u_y = -K*y_LIPM.col(k) + f_Preview*m_ZMP_Reference_Y;
//		y_LIPM.col(k+1) = A*y_LIPM.col(k) + b*u_x;
//	}
//
//}
//
//
//
//

//
//bool PreviewControlWalking::GetRefZMP(int time, matd* pRefXZMP, matd* pRefYZMP)
//{
//	unsigned int step_idx;
//	if(pRefXZMP->size() !=  m_PreviewSize || pRefYZMP->size() != m_PreviewSize ) // �좎떛怨ㅼ삕 �좎룞���좎룞�쇿뜝�섎룄 �좎떖�몄삕?
//		return false;
//	else if(m_StepData.size() == 0) // �좎룞�쇿뜝�숈삕 �좎룞�쇿뜝�숈삕�좑옙�좎룞�쇿뜝�숈삕�좑옙�좎뙐琉꾩삕 �좎룞�쇿뜝占�/		return false;
//	else {
//		//PreviewSize �좎룞�숉겮 For�좎룞�쇿뜝占썲뜝�숈삕�좎듅�먯삕.
//		for(int ref_zmp_idx = 0; ref_zmp_idx < m_PreviewSize;  ref_zmp_idx++)
//		{
//			//time + ref_zmp_idx*TIME_UNIT �좎룞���좎룞�쇿뜝�숈삕�좎룞���좎룞�쇿뜝�숈삕�좎룞���좎떆怨ㅼ삕�좎룞�쇿뜝�숈삕 �у뜝�숈삕 �좎룞���좎룞���좎떇�쎌삕�좑옙�좎룞�쇿뜝�숈삕�좑옙�좎룞�쇿뜝�숈삕
//			if(time + ref_zmp_idx*TIME_UNIT >= m_StepData[m_StepData.size() - 1].TimeData.dAbsStepTime) {
//				(*pRefXZMP)(ref_zmp_idx, 0) = (m_StepData[step_idx].PositionData.stRightFootPosition.x + m_StepData[step_idx].PositionData.stLeftFootPosition.x)*0.5;
//				(*pRefYZMP)(ref_zmp_idx, 0) = (m_StepData[step_idx].PositionData.stRightFootPosition.y + m_StepData[step_idx].PositionData.stLeftFootPosition.y)*0.5;
//			}
//			else {
//				for(step_idx = 0; step_idx < m_StepData.size(); step_idx++)	{
//					if(time + ref_zmp_idx*TIME_UNIT <= m_StepData[step_idx].TimeData.dAbsStepTime)
//						break;
//				}
//
//				if(m_StepData[step_idx].TimeData.bWalkingState == InWalking) {
//					if(m_StepData[step_idx].PositionData.bMovingFoot == LFootMove) {
//						//When  left foot move, the ref. zmp is set avrg. of left foot position
//						(*pRefXZMP)(ref_zmp_idx, 0) = m_StepData[step_idx].PositionData.stRightFootPosition.x + m_X_ZMP_CenterShift;
//						(*pRefYZMP)(ref_zmp_idx, 0) = m_StepData[step_idx].PositionData.stRightFootPosition.y + m_Y_ZMP_CenterShift + m_Y_ZMP_Convergence;
//					}
//					else if(m_StepData[step_idx].PositionData.bMovingFoot == RFootMove) {
//						//When  right foot move, the ref. zmp is set avrg. of left foot position
//						(*pRefXZMP)(ref_zmp_idx, 0) = m_StepData[step_idx].PositionData.stLeftFootPosition.x + m_X_ZMP_CenterShift;
//						(*pRefYZMP)(ref_zmp_idx, 0) = m_StepData[step_idx].PositionData.stLeftFootPosition.y + m_Y_ZMP_CenterShift - m_Y_ZMP_Convergence;
//					}
//					else if(m_StepData[step_idx].PositionData.bMovingFoot == NFootMove) {
//						//When Foot dose not move, the ref. zmp is set avrg. of two foot position
//						(*pRefXZMP)(ref_zmp_idx, 0) = (m_StepData[step_idx].PositionData.stRightFootPosition.x + m_StepData[step_idx].PositionData.stLeftFootPosition.x)*0.5;
//						(*pRefYZMP)(ref_zmp_idx, 0) = (m_StepData[step_idx].PositionData.stRightFootPosition.y + m_StepData[step_idx].PositionData.stLeftFootPosition.y)*0.5;
//					}
//					else
//						 return false;
//
//				}
//				else if(m_StepData[step_idx].TimeData.bWalkingState == InWalkingStarting) {
//					(*pRefXZMP)(ref_zmp_idx, 0) = (m_StepData[step_idx].PositionData.stRightFootPosition.x + m_StepData[step_idx].PositionData.stLeftFootPosition.x)*0.5 + m_X_ZMP_Init;
//					(*pRefYZMP)(ref_zmp_idx, 0) = (m_StepData[step_idx].PositionData.stRightFootPosition.y + m_StepData[step_idx].PositionData.stLeftFootPosition.y)*0.5;
//
//				}
//				else if(m_StepData[step_idx].TimeData.bWalkingState == InWalkingEnding) {
//					(*pRefXZMP)(ref_zmp_idx, 0) = (m_StepData[step_idx].PositionData.stRightFootPosition.x + m_StepData[step_idx].PositionData.stLeftFootPosition.x)*0.5 + m_X_ZMP_Init;
//					(*pRefYZMP)(ref_zmp_idx, 0) = (m_StepData[step_idx].PositionData.stRightFootPosition.y + m_StepData[step_idx].PositionData.stLeftFootPosition.y)*0.5;
//
//				}
//				else
//					return false;
//			}
//		}
//	}
//	return true;
//}

//bool PreviewControlWalking::GetRefZMP(int time, matd* pRefXZMP, matd* pRefYZMP)
//{
//	unsigned int step_idx;
//	if(pRefXZMP->size() !=  m_PreviewSize || pRefYZMP->size() != m_PreviewSize )
//		return false;
//	else {
//		for(step_idx = 0; step_idx < m_StepData.size(); step_idx++)
//		{
//			if(time >= m_StepData[step_idx].TimeData.dAbsStepTime)
//				break;
//		}
//
//		if(m_StepData.size() == 0)
//			return false;
//		else {
//			if(time >= m_StepData[m_StepData.size() - 1].TimeData.dAbsStepTime) {
//				pRefXZMP->fill((m_StepData[m_StepData.size() - 1].PositionData.stRightFootPosition.x + m_StepData[m_StepData.size() - 1].PositionData.stLeftFootPosition.x)*0.5);
//				pRefYZMP->fill((m_StepData[m_StepData.size() - 1].PositionData.stRightFootPosition.y + m_StepData[m_StepData.size() - 1].PositionData.stLeftFootPosition.y)*0.5);
//			}
//			else {
//				for(int ref_zmp_idx = 0; ref_zmp_idx < m_PreviewSize; ref_zmp_idx++)
//				{
//					//k+i(=1~N)
//					if(time + ref_zmp_idx*TIME_UNIT <= m_StepData[step_idx].TimeData.dAbsStepTime) {
//						if(m_StepData[step_idx].TimeData.bWalkingState == InWalking) {
//							if(m_StepData[step_idx].PositionData.bMovingFoot == LFootMove) {
//								//When  left foot move, the ref. zmp is set avrg. of left foot position
//								(*pRefXZMP)(ref_zmp_idx, 0) = m_StepData[step_idx].PositionData.stRightFootPosition.x + m_X_ZMP_CenterShift;
//								(*pRefYZMP)(ref_zmp_idx, 0) = m_StepData[step_idx].PositionData.stRightFootPosition.y + m_Y_ZMP_CenterShift + m_Y_ZMP_Convergence;
//							}
//							else if(m_StepData[step_idx].PositionData.bMovingFoot == RFootMove) {
//								//When  right foot move, the ref. zmp is set avrg. of left foot position
//								(*pRefXZMP)(ref_zmp_idx, 0) = m_StepData[step_idx].PositionData.stLeftFootPosition.x + m_X_ZMP_CenterShift;
//								(*pRefYZMP)(ref_zmp_idx, 0) = m_StepData[step_idx].PositionData.stLeftFootPosition.y + m_Y_ZMP_CenterShift - m_Y_ZMP_Convergence;
//							}
//							else if(m_StepData[step_idx].PositionData.bMovingFoot == NFootMove) {
//								//When Foot dose not move, the ref. zmp is set avrg. of two foot position
//								(*pRefXZMP)(ref_zmp_idx, 0) = (m_StepData[step_idx].PositionData.stRightFootPosition.x + m_StepData[step_idx].PositionData.stLeftFootPosition.x)*0.5;
//								(*pRefYZMP)(ref_zmp_idx, 0) = (m_StepData[step_idx].PositionData.stRightFootPosition.y + m_StepData[step_idx].PositionData.stLeftFootPosition.y)*0.5;
//							}
//							else
//								 return false;
//
//						}
//						else if(m_StepData[step_idx].TimeData.bWalkingState == InWalkingStarting) {
//							if(m_StepData[step_idx].PositionData.bMovingFoot == LFootMove) {
//								//When  left foot move, the ref. zmp is set avrg. of left foot position
//								(*pRefXZMP)(ref_zmp_idx, 0) = m_StepData[step_idx].PositionData.stRightFootPosition.x + m_X_ZMP_Init; //Walking Statig�좎떛�곗삕 Walking Ending �좎떦�곗삕�좎룞��X_ZMP Init�좑옙�좎룞�쇿뜝�숈삕!
//								(*pRefYZMP)(ref_zmp_idx, 0) = m_StepData[step_idx].PositionData.stRightFootPosition.y + m_Y_ZMP_CenterShift + m_Y_ZMP_Convergence;
//							}
//							else if(m_StepData[step_idx].PositionData.bMovingFoot == RFootMove) {
//								//When  right foot move, the ref. zmp is set avrg. of left foot position
//								(*pRefXZMP)(ref_zmp_idx, 0) = m_StepData[step_idx].PositionData.stLeftFootPosition.x + m_X_ZMP_Init;
//								(*pRefYZMP)(ref_zmp_idx, 0) = m_StepData[step_idx].PositionData.stLeftFootPosition.y + m_Y_ZMP_CenterShift - m_Y_ZMP_Convergence;
//							}
//							else
//								return false;
//						}
//						else if(m_StepData[step_idx].TimeData.bWalkingState == InWalkingEnding) {
//							if(m_StepData[step_idx].PositionData.bMovingFoot == LFootMove) {
//								//When  left foot move, the ref. zmp is set avrg. of left foot position
//								(*pRefXZMP)(ref_zmp_idx, 0) = m_StepData[step_idx].PositionData.stRightFootPosition.x + m_X_ZMP_Init;
//								(*pRefYZMP)(ref_zmp_idx, 0) = m_StepData[step_idx].PositionData.stRightFootPosition.y + m_Y_ZMP_CenterShift + m_Y_ZMP_Convergence;
//							}
//							else if(m_StepData[step_idx].PositionData.bMovingFoot == RFootMove) {
//								//When  right foot move, the ref. zmp is set avrg. of left foot position
//								(*pRefXZMP)(ref_zmp_idx, 0) = m_StepData[step_idx].PositionData.stLeftFootPosition.x + m_X_ZMP_Init;
//								(*pRefYZMP)(ref_zmp_idx, 0) = m_StepData[step_idx].PositionData.stLeftFootPosition.y + m_Y_ZMP_CenterShift - m_Y_ZMP_Convergence;
//							}
//							else
//								return false;
//						}
//					}
//					else {
//						step_idx += 1;
//						if(step_idx > m_StepData.size() - 1)
//						{
//							step_idx -= 1;
//							(*pRefXZMP)(ref_zmp_idx, 0) = (m_StepData[step_idx].PositionData.stRightFootPosition.x + m_StepData[step_idx].PositionData.stLeftFootPosition.x)*0.5;
//							(*pRefYZMP)(ref_zmp_idx, 0) = (m_StepData[step_idx].PositionData.stRightFootPosition.y + m_StepData[step_idx].PositionData.stLeftFootPosition.y)*0.5;
//						}
//						else {
//							if(m_StepData[step_idx].TimeData.bWalkingState == InWalking) {
//								if(m_StepData[step_idx].PositionData.bMovingFoot == LFootMove) {
//									//When  left foot move, the ref. zmp is set avrg. of left foot position
//									(*pRefXZMP)(ref_zmp_idx, 0) = m_StepData[step_idx].PositionData.stRightFootPosition.x + m_X_ZMP_CenterShift;
//									(*pRefYZMP)(ref_zmp_idx, 0) = m_StepData[step_idx].PositionData.stRightFootPosition.y + m_Y_ZMP_CenterShift + m_Y_ZMP_Convergence;
//								}
//								else if(m_StepData[step_idx].PositionData.bMovingFoot == RFootMove) {
//									//When  right foot move, the ref. zmp is set avrg. of left foot position
//									(*pRefXZMP)(ref_zmp_idx, 0) = m_StepData[step_idx].PositionData.stLeftFootPosition.x + m_X_ZMP_CenterShift;
//									(*pRefYZMP)(ref_zmp_idx, 0) = m_StepData[step_idx].PositionData.stLeftFootPosition.y + m_Y_ZMP_CenterShift - m_Y_ZMP_Convergence;
//								}
//								else if(m_StepData[step_idx].PositionData.bMovingFoot == NFootMove) {
//									//When Foot dose not move, the ref. zmp is set avrg. of two foot position
//									(*pRefXZMP)(ref_zmp_idx, 0) = (m_StepData[step_idx].PositionData.stRightFootPosition.x + m_StepData[step_idx].PositionData.stLeftFootPosition.x)*0.5;
//									(*pRefYZMP)(ref_zmp_idx, 0) = (m_StepData[step_idx].PositionData.stRightFootPosition.y + m_StepData[step_idx].PositionData.stLeftFootPosition.y)*0.5;
//								}
//								else
//									 return false;
//
//							}
//							else if(m_StepData[step_idx].TimeData.bWalkingState == InWalkingStarting) {
//								if(m_StepData[step_idx].PositionData.bMovingFoot == LFootMove) {
//									//When  left foot move, the ref. zmp is set avrg. of left foot position
//									(*pRefXZMP)(ref_zmp_idx, 0) = m_StepData[step_idx].PositionData.stRightFootPosition.x + m_X_ZMP_Init; //Walking Statig�좎떛�곗삕 Walking Ending �좎떦�곗삕�좎룞��X_ZMP Init�좑옙�좎룞�쇿뜝�숈삕!
//									(*pRefYZMP)(ref_zmp_idx, 0) = m_StepData[step_idx].PositionData.stRightFootPosition.y + m_Y_ZMP_CenterShift + m_Y_ZMP_Convergence;
//								}
//								else if(m_StepData[step_idx].PositionData.bMovingFoot == RFootMove) {
//									//When  right foot move, the ref. zmp is set avrg. of left foot position
//									(*pRefXZMP)(ref_zmp_idx, 0) = m_StepData[step_idx].PositionData.stLeftFootPosition.x + m_X_ZMP_Init;
//									(*pRefYZMP)(ref_zmp_idx, 0) = m_StepData[step_idx].PositionData.stLeftFootPosition.y + m_Y_ZMP_CenterShift - m_Y_ZMP_Convergence;
//								}
//								else
//									return false;
//							}
//							else if(m_StepData[step_idx].TimeData.bWalkingState == InWalkingEnding) {
//								if(m_StepData[step_idx].PositionData.bMovingFoot == LFootMove) {
//									//When  left foot move, the ref. zmp is set avrg. of left foot position
//									(*pRefXZMP)(ref_zmp_idx, 0) = m_StepData[step_idx].PositionData.stRightFootPosition.x + m_X_ZMP_Init;
//									(*pRefYZMP)(ref_zmp_idx, 0) = m_StepData[step_idx].PositionData.stRightFootPosition.y + m_Y_ZMP_CenterShift + m_Y_ZMP_Convergence;
//								}
//								else if(m_StepData[step_idx].PositionData.bMovingFoot == RFootMove) {
//									//When  right foot move, the ref. zmp is set avrg. of left foot position
//									(*pRefXZMP)(ref_zmp_idx, 0) = m_StepData[step_idx].PositionData.stLeftFootPosition.x + m_X_ZMP_Init;
//									(*pRefYZMP)(ref_zmp_idx, 0) = m_StepData[step_idx].PositionData.stLeftFootPosition.y + m_Y_ZMP_CenterShift - m_Y_ZMP_Convergence;
//								}
//								else
//									return false;
//							}
//
//						}
//
//					}
//				}
//			}
//		}
//	}
//	return true;
//}
