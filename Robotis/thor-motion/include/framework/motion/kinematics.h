/*
 * kinematics.h
 *
 *  Created on: 2013. 2. 4.
 *      Author: hjsong
 */

#ifndef KINEMATICS_H_
#define KINEMATICS_H_

#include "math/matrix.h"
#include "motion/jointdata.h"

namespace Thor
{
	class Kinematics
	{
	private:
		static Kinematics* m_UniqueInstance;
        Kinematics();

	protected:

	public:
		static const double CAMERA_DISTANCE = 33.2; //mm
		static const double EYE_TILT_OFFSET_ANGLE = 0.0;//40.0; //degree
		static const double LEG_SIDE_OFFSET = 144.0; //mm
		static const double THIGH_LENGTH = 301.49627; //mm
		static const double CALF_LENGTH = 301.49627; //mm
		static const double ANKLE_LENGTH = 118.0; //mm
		static const double LEG_LENGTH = 720.99254; //mm (THIGH_LENGTH + CALF_LENGTH + ANKLE_LENGTH)

		~Kinematics();

		static Kinematics* GetInstance()			{ return m_UniqueInstance; }
	};
}


#endif /* KINEMATICS_H_ */
