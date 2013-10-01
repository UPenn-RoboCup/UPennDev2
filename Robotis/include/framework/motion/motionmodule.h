/*
 * motionmodule.h
 *
 *  Created on: 2013. 1. 11.
 *      Author: zerom
 */

#ifndef MOTIONMODULE_H_
#define MOTIONMODULE_H_

#include "jointdata.h"
#include <vector>

namespace Thor
{

class MotionModule
{

public:
	char *uID;
	std::vector<JointData> m_RobotInfo;
	static const int TIME_UNIT = 8; // 8msec

	virtual void Initialize() = 0;
	virtual void Process() = 0;

};

} /* namespace Thor */
#endif /* MOTIONMODULE_H_ */
