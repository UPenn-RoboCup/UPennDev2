/*
 * dynamic_walking.h
 *
 *  Created on: 2013. 10. 7.
 *      Author: hjsong
 */

#ifndef DYNAMIC_WALKING_H_
#define DYNAMIC_WALKING_H_

#include <string.h>

//#include "minIni.h"
#include "framework/motion/motionmodule.h"
#include "motion/motionstatus.h"

#define DYNAMICS_WALKING_SECTION "Dyanmic Walking Config"

namespace Thor
{
	class DynamicWalking : public MotionModule
	{
	public:
		enum
		{
			PHASE0 = 0,
			PHASE1 = 1,
			PHASE2 = 2,
			PHASE3 = 3
		};

	private:
        DynamicWalking();
		static DynamicWalking* m_UniqueInstance;
		int data_index;
		bool m_Real_Running;


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

		double m_roll_init_rad;
		double m_pitch_init_rad;
		double m_yaw_init_rad;


		double* m_right_leg_ft_fz_N_array;
		double* m_left_leg_ft_fz_N_array;

	public:

		virtual ~DynamicWalking();

		int   FZ_WINDOW_SIZE;

		bool   DEBUG_PRINT;
		bool   BALANCE_ENABLE;
		double BALANCE_KNEE_GAIN;
		double BALANCE_ANKLE_PITCH_GAIN;
		double BALANCE_HIP_ROLL_GAIN;
		double BALANCE_ANKLE_ROLL_GAIN;

		double HIP_PITCH_OFFSET;


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

		double idxIncreasement;

		int    P_GAIN;
		int    I_GAIN;
		int    D_GAIN;

		const char* m_AngleData_File_Name;
		const char* m_EndPointeData_File_Name;
		const char* m_BalancingIndexeData_File_Name;


		double m_LastAngle[16];

		void Initialize();
		void ReInitialize();
		void SetFileName(const char* AngleData_File_Name, const char* EndPointeData_File_Name, const char* BalancingIndexeData_File_Name);
		void SetInitAngleinRad(double roll_init_rad, double pitch_init_rad);


		double AngleData[200000];
		double EndPointData[200000];
		int BalancingIdxData[200000];

		bool computeIK(double *out, double x, double y, double z, double a, double b, double c);

		static DynamicWalking* GetInstance() { return m_UniqueInstance; }

		void Process();
		void Start();
		void Stop();

	};

}




#endif /* DYNAMIC_WALKING_H_ */
