/*
 *   Walking.h
 *
 *   Author: ROBOTIS
 *
 */

#ifndef _RECURSIVE_WALKING_ENGINE_H_
#define _RECURSIVE_WALKING_ENGINE_H_

//#define WEBOT_SIMULATION
#define REAL_ROBOT

#ifdef WEBOT_SIMULATION

#include <stdio.h>
#include <string.h>
#include <vector>
#include <iostream>
//#include "minIni.h"
#include "MotionModule.h"
#include "LinearAlgebra.h"

#else
#include <stdio.h>
#include <string.h>
#include <vector>
#include <iostream>
#include "minIni.h"
#include "framework/motion/motionmodule.h"
#include "framework/motion/motionstatus.h"
#include "framework/math/linear_algebra.h"
#endif

#define DYNAMICS_WALKING_SECTION "Dyanmic Walking Config"

#ifdef WEBOT_SIMULATION
namespace Robot
{
#else
namespace Thor
{
#endif
	class RecursiveWalking : public MotionModule
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
		RecursiveWalking();
		static RecursiveWalking* m_UniqueInstance;
		
		std::vector<StepData> m_StepData;

		Pose3D m_PresentGtoRightFootPosition;
		Pose3D m_PresentGtoLeftFootPosition;
		Pose3D m_PresentGtoBodyPosition;
	public:
		Pose3D m_ReferenceGtoBodyPosition;
		Pose3D m_ReferenceGtoRightFootPosition;
		Pose3D m_ReferenceGtoLeftFootPosition;
	private:
//		Pose3D m_COBtoPresentRightFootPosition;
//		Pose3D m_COBtoPresentLeftFootPosition;
//		Pose3D m_COBtoPresentBodyPosition;
		//bool m_Real_Running, m_Ctrl_Running;
		bool m_Real_Running, m_Ctrl_Running;
		bool m_IsCompleteCalcPattern;

		int m_Time;         //RelativeTime
		int m_WalkingTime;  //Absolute Time
		int m_ReferenceTime;//Absolute Time

		int m_PresentStepNum;
		int m_PreviousStepNum;
		veci m_Balancing_Idx;

		//These parameters are need for walking calculation
		double m_PeriodTime;
		double m_DSP_Ratio;
		double m_SSP_Ratio;
		double m_Foot_Move_PeriodTime;
		double m_Body_Move_PeriodTime;

		double m_SSP_Time;
		double m_SSP_Time_Start;
		double m_SSP_Time_End;

		matd matCOBtoG;
		matd matGtoCOB;
		matd matCOBtoRH;
		matd matCOBtoLH;
		matd matRHtoCOB;
		matd matLHtoCOB;
		matd matRHtoRF;
		matd matLHtoLF;
		matd matGtoRF;
		matd matGtoLF;


		//These matrix and parameters are for preview control
		int m_PatternDataSize;
		veci m_StepIdxData;
		veci m_DetailTimeIdx;
		matd A, IA;

		matd x_LIPM, y_LIPM;
		matd x_MPMM, y_MPMM;
//		matd x_MPMM_pre, y_MPMM_pre;
//		matd x_MPMM_post, y_MPMM_post;
		matd x_delta, y_delta;

		matd m_ZMP_Reference_X, m_ZMP_Reference_Y;
		matd m_ZMP_Calculated_by_MPMM_X, m_ZMP_Calculated_by_MPMM_Y;
//		matd m_ZMP_Calculated_by_MPMM_X_pre, m_ZMP_Calculated_by_MPMM_Y_pre;
//		matd m_ZMP_Calculated_by_MPMM_X_post, m_ZMP_Calculated_by_MPMM_Y_post;

		std::vector<matd> m_matvGtoRF, m_matvGtoLF;
		std::vector<matd> m_matvGtoCOB;
		vecd m_ShoulderSwingGain, m_ElbowSwingGain;

		std::vector<matd> m_matvGtoRFforPlaying, m_matvGtoLFforPlaying;
		std::vector<matd> m_matvGtoCOBforPlaying;
		vecd m_ShoulderSwingGainforPlaying, m_ElbowSwingGainforPlaying;

		matd PData1, PData2, PData3;

		double wsin(double time, double period, double period_shift, double mag, double mag_shift);
		double wsigmoid(double time, double period, double time_shift, double mag, double mag_shift, double sigmoid_ratio, double distortion_ratio);

		//
		void CalcDetailTimeIdx();
		void CalcIdxData();
		void CalcRefZMP();
		void CalcEndPointData();
		void CalcCOBData(matd x_COB, matd y_COB);
		bool CalcZMPbyMPMM();

		//
		int dir[16];
		int dir_output[16];
		double InitAngle[16];
		double r_arm[6], r_arm_init[6];
		double l_arm[6], l_arm_init[6];

		int m_play_idx;

		double m_dplay_idx;

		void CalcDHforRightArm(double *rightArmAngle_deg, matd *T01, matd *T02, matd *T03, matd *T04, matd *T05, matd *T06);
		void CalcDHforLeftArm(double *leftArmAngle_deg, matd *T01, matd *T02, matd *T03, matd *T04, matd *T05, matd *T06);
		void CalcDHforRightLeg(double *rightLegAngle_deg, matd *T01, matd *T02, matd *T03, matd *T04, matd *T05, matd *T06);
		void CalcDHforLeftLeg(double *leftLegAngle_deg, matd *T01, matd *T02, matd *T03, matd *T04, matd *T05, matd *T06);

		matd GetCOMofMultiBody(double* leftLegAngle_deg, double* rightLegAngle_deg, double* leftArmAngle_deg, double* rightArmAngle_deg);

	public:
#ifdef WEBOT_SIMULATION
		double Webot_Rad[16];
#else
		int outValue[16];
#endif

		double m_X_ZMP_Init, m_X_ZMP_CenterShift;
		double m_Y_ZMP_Convergence, m_Y_ZMP_CenterShift;

		// Walking initial pose
		double X_Offset;
		double Y_OFfset;
		double Z_Offset;
		double A_Offset;
		double B_Offset;
		double C_Offset;

		//Position Gain of DXL Pro
		int    P_GAIN;
		int    I_GAIN;
		int    D_GAIN;

		bool DEBUG_PRINT;

		//Balancing
		bool BALANCE_ENABLE;
		double HIP_PITCH_OFFSET;
#ifdef REAL_ROBOT
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
#endif

		virtual ~RecursiveWalking();

		bool computeIK(double *out, double x, double y, double z, double a, double b, double c);

		static RecursiveWalking* GetInstance() { return m_UniqueInstance; }
		void Initialize();
		void Start();
		void Stop();
		void Process();
		bool IsRunning();

		void AddStepData(StepData step_data);
		bool CalcWalkingPattern();
		std::vector<StepData> GetStepData();

//		int GetTotalTime();
//		int GetPresentTime();
		void GetPresentReferencePose(Pose3D *present_body, Pose3D *present_right_foot, Pose3D *present_left_foot);

		void SetInitAngleinRad(double roll_init_rad, double pitch_init_rad);
		void SetRefZMPDecisionParameter(double X_ZMP_CenterShift, double Y_ZMP_CenterShift, double Y_ZMP_Convergence);
		void SetArmAngle(double *leftLegAngle_rad, double *rightLegAngle_rad);

		void ClearStepData();
	};
}

#endif
