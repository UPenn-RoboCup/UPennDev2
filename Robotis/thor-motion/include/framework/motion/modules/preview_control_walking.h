/*
 *   Walking.h
 *
 *   Author: ROBOTIS
 *
 */

#ifndef _DYNAMIC_WALKING_ENGINE_H_
#define _DYNAMIC_WALKING_ENGINE_H_

#include <stdio.h>
#include <string.h>
#include <vector>
#include <iostream>
//#include "minIni.h"
#include "framework/motion/motionmodule.h"
#include "framework/motion/motionstatus.h"
#include "framework/math/linear_algebra.h"

#define DYNAMICS_WALKING_SECTION "Dyanmic Walking Config"
#define REAL_ROBOT
//#define WEBOT_SIMULATION

namespace Thor
{
	class PreviewControlWalking : public MotionModule
	{
	public:
		enum
		{
			BALANCING_PHASE0 = 0,
			BALANCING_PHASE1 = 1,
			BALANCING_PHASE2 = 2,
			BALANCING_PHASE3 = 3,
			BALANCING_PHASE4 = 4,
			BALANCING_PHASE5 = 5,
			BALANCING_PHASE6 = 6,
			BALANCING_PHASE7 = 7,
			BALANCING_PHASE8 = 8,
			BALANCING_PHASE9 = 9
		};

	private:
		PreviewControlWalking();
		static PreviewControlWalking* m_UniqueInstance;
		
		std::vector<StepData> m_StepData;

		Pose3D m_PresentRightFootPosition;
		Pose3D m_PresentLeftFootPosition;
		Pose3D m_PresentBodyPosition;

		bool m_Real_Running, m_Ctrl_Running;

		int m_Time;         //RelativeTime
		int m_WalkingTime;  //Absolute Time
		int m_ReferenceTime;//Absolute Time

		int m_PresentStepNum;
		int m_PreviousStepNum;
		int m_Balancing_Index;

		//These parameters are need for walking calculation
		double m_PeriodTime;
		double m_DSP_Ratio;
		double m_SSP_Ratio;
		double m_Foot_Move_PeriodTime;
		double m_Body_Move_PeriodTime;

		double m_SSP_Time;
		double m_SSP_Time_Start;
		double m_SSP_Time_End;

		Pose3D m_ReferenceRightFootPosition;
		Pose3D m_ReferenceLeftFootPosition;
		Pose3D m_ReferenceBodyPosition;

		matd matCOBtoG;
		matd matGtoCOB;
		matd matCOBtoRH;
		matd matCOBtoLH;
		matd matRHtoCOB;
		matd matLHtoCOB;
		matd matRHtoRF;
		matd matLHtoLF;


		///////////////////////////////////////////////////////////
		//Time for Preview Control and Dynamics Regulator
		double m_PreivewTime;
		double m_DynamicsFilterTime;

		//These matrix and parameters are for preview control
		matd A, b,c;
		matd K, P; // result of riccati eqn
		matd f_Preview, f_Dfilter;
		int m_PreviewSize, m_DFilterSize;

		matd u_x, u_y;
		matd u_delta_x, u_delta_y;

		matd m_StepIdxforCalc;
		matd x_LIPM, y_LIPM;
		matd x_MPMM_pre, y_MPMM_pre;
		//matd x_MPMM_post, y_MPMM_post;
		matd x_delta, y_delta;

		matd m_ZMP_Reference_X, m_ZMP_Reference_Y;
		matd m_ZMP_Generated_by_LIPM_X, m_ZMP_Generated_by_LIPM_Y;
		matd m_ZMP_Calculated_by_MPMM_X_pre, m_ZMP_Calculated_by_MPMM_Y_pre;
		matd m_ZMP_Calculated_by_MPMM_X_post, m_ZMP_Calculated_by_MPMM_Y_post;

		veci m_StepIdxData;
		std::vector<matd> m_matvGtoRF, m_matvGtoLF;
		std::vector<matd> m_matvGtoCOB;

		matd PData1, PData2, PData3;
		matd FilterdPData1, FilterdPData2, FilterdPData3;

		double wsin(double time, double period, double period_shift, double mag, double mag_shift);
		double wsigmoid(double time, double period, double time_shift, double mag, double mag_shift, double sigmoid_ratio, double distortion_ratio);

		double m_X_ZMP_Init, m_X_ZMP_CenterShift;
		double m_Y_ZMP_Convergence, m_Y_ZMP_CenterShift;

		//�좎룞�쇿뜝�숈삕�좎룞���좎룞�쇿뜝�숈삕�좎룞���좎떗�몄삕�좎떦紐뚯삕 �좎룞�쇿뜝�숈삕 �좎룞�쇿뜝�숈삕�좑옙�좎떗�몄삕�좎룞���좎룞��
		void CalcStepIdxData();
		void CalcRefZMP();
		void CalcEndPointData();
		void CalcCOBData(matd x_COB, matd y_COB);
		bool CalcZMPbyMPMM();
		//bool GetRefZMP(int time, matd* pRefXZMP, matd* pRefYZMP);
		//bool GetEndPointData(double cob_x, double cob_y, int time, matd* pmatGtoCOB, matd* pmatRHtoRF, matd* pmatLHtoLF);

		//�좎떬諭꾩삕�좎룞�쇿뜝�꾩뿉 �좎떗�몄삕�좎룞���좎뛿�앭뜝�숈삕
		int dir[16];
		int dir_output[16];
		double InitAngle[16];
		double r_arm[6], r_arm_init[6];
		double l_arm[6], l_arm_init[6];

		void CalcDHforRightArm(double *rightArmAngle_deg, matd *T01, matd *T02, matd *T03, matd *T04, matd *T05, matd *T06);
		void CalcDHforLeftArm(double *leftArmAngle_deg, matd *T01, matd *T02, matd *T03, matd *T04, matd *T05, matd *T06);
		void CalcDHforRightLeg(double *rightLegAngle_deg, matd *T01, matd *T02, matd *T03, matd *T04, matd *T05, matd *T06);
		void CalcDHforLeftLeg(double *leftLegAngle_deg, matd *T01, matd *T02, matd *T03, matd *T04, matd *T05, matd *T06);

		matd GetCOMofMultiBody(double* leftLegAngle_deg, double* rightLegAngle_deg, double* leftArmAngle_deg, double* rightArmAngle_deg);

		//for balancing
		double foot_landing_detection_time_sec;

		double foot_r_roll_landing_offset_rad;
		double foot_r_pitch_landing_offset_rad;
		double foot_l_roll_landing_offset_rad;
		double foot_l_pitch_landing_offset_rad;

		double foot_r_roll_adjustment_rad;
		double foot_r_pitch_adjustment_rad;
		double foot_l_roll_adjustment_rad;
		double foot_l_pitch_adjustment_rad;

		double cob_x_adjustment_mm;
		double cob_y_adjustment_mm;

		double gyro_roll_init_rad_per_sec;
		double gyro_pitch_init_rad_per_sec;
		double gyro_yaw_init_rad_per_sec;

		double m_iu_roll_init_rad;
		double m_iu_pitch_init_rad;
		double m_iu_yaw_init_rad;

		double* m_right_leg_ft_fz_N_array;
		double* m_left_leg_ft_fz_N_array;


	public:
		double Webot_Rad[16];

		// Walking initial pose
		double X_Offset;
		double Y_OFfset;
		double Z_Offset;
		double A_Offset;
		double B_Offset;
		double C_Offset;

		// Walking control
		double PERIOD_TIME;
		double DSP_RATIO;

		//Position Gain of DXL Pro
		int    P_GAIN;
		int    I_GAIN;
		int    D_GAIN;

		bool DEBUG_PRINT;
		bool FILTERING_ENABLE;


		//Balancing
		bool BALANCE_ENABLE;
		double HIP_PITCH_OFFSET;

		//Balancing Gain
		double BALANCE_KNEE_GAIN;
		double BALANCE_ANKLE_PITCH_GAIN;
		double BALANCE_HIP_ROLL_GAIN;
		double BALANCE_ANKLE_ROLL_GAIN;

		double WALK_STABILIZER_GAIN_RATIO;
		double IMU_GYRO_GAIN_RATIO;
		double FORCE_MOMENT_DISTRIBUTION_RATIO;

		double BALANCE_X_GAIN;
		double BALANCE_Y_GAIN;
		double BALANCE_PITCH_GAIN;
		double BALANCE_ROLL_GAIN;

		double FOOT_LANDING_OFFSET_GAIN;
		double FOOT_LANDING_DETECT_N;

		double SYSTEM_CONTROL_UNIT_TIME_SEC;
		double FOOT_LANDING_DETECTION_TIME_MAX_SEC;

		double FOOT_ROLL_ADJUSTMENT_ABS_MAX_RAD;
		double FOOT_PITCH_ADJUSTMENT_ABS_MAX_RAD;

		double COB_X_ADJUSTMENT_ABS_MAX_MM;
		double COB_Y_ADJUSTMENT_ABS_MAX_MM;



		virtual ~PreviewControlWalking();

		bool computeIK(double *out, double x, double y, double z, double a, double b, double c);

		static PreviewControlWalking* GetInstance() { return m_UniqueInstance; }
		void Initialize();
		void Start();
		void Stop();
		void Process();
		bool IsRunning();

		void AddStepData(StepData step_data);
		void SetInitAngleinRad(double roll_init_rad, double pitch_init_rad);
		void SetSizeforPreviewControl(double previewTime_sec, double dynamicsFilterTime_sec);
		void SetRefZMPDecisionParameter(double X_ZMP_CenterShift, double Y_ZMP_CenterShift, double Y_ZMP_Convergence);
		void SetArmAngle(double *leftLegAngle_rad, double *rightLegAngle_rad);

		//void ClearStepData();


		//void GetPresentPositionInfo();
	};

}
//namespace Robot
//{
//
//	typedef struct
//	{
//		int bMovingFoot;
//		double dFootHeight, dZ_Swap_Amplitude, dShoulderSwingGain, dElbowSwingGain;
//		double dWaistPichAngle, dWaistYawAngle;
//		Pose3D stLeftFootPosition;
//		Pose3D stRightFootPosition;
//		Pose3D stBodyPosition;
//	} StepPositionData;
//
//	typedef struct
//	{
//		int bWalkingState;
//		double dAbsStepTime, dDSPratio;
//		double sigmoid_ratio_x, sigmoid_ratio_y, sigmoid_ratio_z, sigmoid_ratio_roll, sigmoid_ratio_pitch, sigmoid_ratio_yaw;
//		double sigmoid_distortion_x, sigmoid_distortion_y, sigmoid_distortion_z, sigmoid_distortion_roll, sigmoid_distortion_pitch, sigmoid_distortion_yaw;
//	} StepTimeData;
//
//	typedef struct
//	{
//		StepPositionData PositionData;
//		StepTimeData TimeData;
//	} StepData;
//
//
//
//	class PreviewControlWalking : public MotionModule
//	{
//	public:
//		enum
//		{
//			PHASE0 = 0,
//			PHASE1 = 1,
//			PHASE2 = 2,
//			PHASE3 = 3
//		};
//
//	private:
//		PreviewControlWalking();
//		static PreviewControlWalking* m_UniqueInstance;
//
//		std::vector<StepData> m_StepData;
//		Pose3D m_PresentRightPosition;
//		Pose3D m_PresentLeftPosition;
//		Pose3D m_PresentBodyPosition;
//
//		bool m_Real_Running, m_Ctrl_Running;
//
//
//		int m_WalkingTime; //Absolute Time
//		int m_PresentStepNum;
//		int m_PreviousStepNum;
//		int m_Balancing_Index;
//
//		//Time for Preview Control and Dynamics Regulator
//		double m_PreivewTime;
//		double m_DynamicsFilterTime;
//
//		//These parameters are need for walking calculation
//		double m_PeriodTime;
//		double m_DSP_Ratio;
//		double m_SSP_Ratio;
//		double m_Foot_Move_PeriodTime;
//		double m_Body_Move_PeriodTime;
//
//		double m_SSP_Time;
//		double m_SSP_Time_Start;
//		double m_SSP_Time_End;
//
//		//These matrix and parameters are for preview control
//		matd A, b,c;
//		matd K, P; // result of riccati eqn
//		matd f_Preview, f_Dfilter;
//		int m_PreviewSize, m_DFilterSize;
//
//		matd x_LIPM, y_LIPM;
//		matd x_MPMM_pre, y_MPMM_pre;
//		matd x_MPMM_post, y_MPMM_post;
//		matd x_delta, y_delta;
//
//		matd m_ZMP_Reference_X, m_ZMP_Reference_Y;
//		matd m_ZMP_Generated_by_LIPM_X, m_ZMP_Generated_by_LIPM_Y;
//		matd m_ZMP_Calculated_by_MPMM_X_pre, m_ZMP_Calculated_by_MPMM_Y_pre;
//		matd m_ZMP_Calculated_by_MPMM_X_post, m_ZMP_Calculated_by_MPMM_Y_post;
//
//		matd PData1, PData2, PData3;
//
//		double wsin(double time, double period, double period_shift, double mag, double mag_shift);
//		double wsigmoid(double time, double period, double time_shift, double mag, double mag_shift, double sigmoid_ratio, double distortion_ratio);
//
//
//		double m_X_ZMP_Init, m_X_ZMP_CenterShift;
//		double m_Y_ZMP_Convergence, m_Y_ZMP_CenterShift;
//
//
//		void RefreshStepData();
//		void CheckPresentStepNum();
//
//	public:
//		// Walking initial pose
//		double X_Offset;
//		double Y_OFfset;
//		double Z_Offset;
//		double A_Offset;
//		double B_Offset;
//		double C_Offset;
//
//		// Walking control
//		double PERIOD_TIME;
//		double DSP_RATIO;
//
//		// Balance control
//		double HIP_PITCH_OFFSET;
//
//		//Position Gain of DXL Pro
//		int    P_GAIN;
//		int    I_GAIN;
//		int    D_GAIN;
//
//		virtual ~PreviewControlWalking();
//
//		bool computeIK(double *out, double x, double y, double z, double a, double b, double c);
//
//		static PreviewControlWalking* GetInstance() { return m_UniqueInstance; }
//		void Initialize();
//		void Start();
//		void Stop();
//		void Process();
//		bool IsRunning();
//
//
//		void AddStepData(StepData step_data);
//		//void ClearStepData();
//
//		bool GetRefZMP(int time, matd* pRefXZMP, matd* pRefYZMP);
//		void GetEndPointData(double cob_x, double cob_y, int time, matd* pmatGtoCOB, matd* pmatRHtoRF, matd* pmatLHtoLF);
//		//void GetPresentPositionInfo();
//
//		void SetRefZMPDecisionParameter(double X_ZMP_CenterShift, double Y_ZMP_CenterShift, double Y_ZMP_Convergence);
//		void SetSizeforPreviewControl(double previewTime_sec, double dynamicsFilterTime_sec);
//
//		void CalcDHforRightArm();
//		void CalcDHforLeftArm();
//		void CalcDHforRightLeg();
//		void CalcDHforLeftLeg();
//
//		void GetCOMofMultiBody(double* leftLegAngle_rad, double* rightLegAngle_rad, double* leftArmAngle_rad, double* rightArmAngle_rad, Pose3D* pCOMPositionList);
//
//	};
//
//}

#endif
