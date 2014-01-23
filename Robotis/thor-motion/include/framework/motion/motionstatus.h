/*
 * motionstatus.h
 *
 *  Created on: 2013. 1. 21.
 *      Author: hjsong
 */

#ifndef MOTIONSTATUS_H_
#define MOTIONSTATUS_H_

#include <vector>
#include <string.h>
#include "jointdata.h"

namespace Thor
{
    enum
	{
        BACKWARD    = -1,
        STANDUP     = 0,
        FORWARD     = 1
    };

	class EnableList
	{
	public:
		char *uID;

		EnableList() : uID("Test") {	}

		bool IsEqual(const char *uID)
		{
			if(strcmp(this->uID, uID) == 0 )
				return true;
			else
				return false;
		}

		~EnableList()
		{		}
	};



	class MotionStatus
	{
	private:

	public:
		static std::vector<JointData> m_CurrentJoints;
		static const int MAXIMUM_NUMBER_OF_JOINTS = 38;
		static EnableList m_EnableList[MAXIMUM_NUMBER_OF_JOINTS];
		static double FB_GYRO;
		static double RL_GYRO;
		static double FB_ACCEL;
		static double RL_ACCEL;
		static double EulerAngleX,EulerAngleY,EulerAngleZ;

		static int FALLEN;

		static double R_LEG_FSR1, R_LEG_FSR2, R_LEG_FSR3, R_LEG_FSR4;
		static double L_LEG_FSR1, L_LEG_FSR2, L_LEG_FSR3, L_LEG_FSR4;

		static double R_LEG_FX, R_LEG_FY, R_LEG_FZ, R_LEG_TX, R_LEG_TY, R_LEG_TZ;
		static double L_LEG_FX, L_LEG_FY, L_LEG_FZ, L_LEG_TX, L_LEG_TY, L_LEG_TZ;

		static double R_ARM_FSR1, R_ARM_FSR2, R_ARM_FSR3, R_ARM_FSR4;
		static double L_ARM_FSR1, L_ARM_FSR2, L_ARM_FSR3, L_ARM_FSR4;

		static double R_ARM_FX, R_ARM_FY, R_ARM_FZ, R_ARM_TX, R_ARM_TY, R_ARM_TZ;
		static double L_ARM_FX, L_ARM_FY, L_ARM_FZ, L_ARM_TX, L_ARM_TY, L_ARM_TZ;

		static double lidar_data[2][1080];
	};
}


#endif /* MOTIONSTATUS_H_ */
