/*
 * testmodule.h
 *
 *  Created on: 2013. 1. 24.
 *      Author: hjsong
 */

#ifndef TESTMODULE_H_
#define TESTMODULE_H_

#include "framework/motion/motionmodule.h"
#include <time.h>

namespace Thor
{

class Test : public MotionModule
{
private:
	static Test* m_UniqueInstance;
	Test();
	int m_Count;
	time_t startTime, endTime;

public:
	static Test* GetInstance() { return m_UniqueInstance; }
	void Initialize();
	void Process();

	void moveFirstPose();
	void moveSecondPose();
	void moveThirdPose();
	void moveFourthPose();
	void moveFifthPose();
	void moveSixthPose();
	void moveSeventhPose();
	void moveEighthPose();
	void moveNinthPose();
	void moveTenthPose();
	void moveEleventhPose();
	void moveTwelfthPose();
	void moveThirteenthPose();
	void moveFourteenthPose();

	~Test() { }
};


}


#endif /* TESTMODULE_H_ */
