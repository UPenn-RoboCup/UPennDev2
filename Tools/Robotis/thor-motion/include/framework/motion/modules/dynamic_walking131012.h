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
	class DynamicWalking131012 : public MotionModule
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
		DynamicWalking131012();
		static DynamicWalking131012* m_UniqueInstance;
		int i ;
	public:

		virtual ~DynamicWalking131012();

		bool   BALANCE_ENABLE;
		double BALANCE_KNEE_GAIN;
		double BALANCE_ANKLE_PITCH_GAIN;
		double BALANCE_HIP_ROLL_GAIN;
		double BALANCE_ANKLE_ROLL_GAIN;

		double HIP_PITCH_OFFSET;

		int    P_GAIN;
		int    I_GAIN;
		int    D_GAIN;

		const char* m_File_Name;

		double m_LastAngle[16];

		void Initialize();
		void ReInitialize();
		void SetFileName(const char* file_name);


		double AngleData[50000];
		bool computeIK(double *out, double x, double y, double z, double a, double b, double c);

		static DynamicWalking131012* GetInstance() { return m_UniqueInstance; }

		void Process();

	};

}




#endif /* DYNAMIC_WALKING_H_ */
