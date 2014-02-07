/*
 * testmodule.cpp
 *
 *  Created on: 2013. 1. 24.
 *      Author: hjsong
 */

#include <stdio.h>
#include <string.h>
#include "framework/motion/modules/testmodule.h"
#include "framework/motion/motionstatus.h"


using namespace Thor;

Test* Test::m_UniqueInstance = new Test();


Test::Test() : m_Count(0)
{
	startTime = endTime = 0;
	uID = "Test";
}

void Test::Initialize()
{
	if(MotionStatus::m_CurrentJoints.size() != 0)
		m_RobotInfo = MotionStatus::m_CurrentJoints;
	else
		fprintf(stderr,"MotionStatus is not initialized");
}

void Test::Process()
{

//	if(m_Count == 0)
//	{
//		time(&startTime);
//		//moveFirstPose();
//		m_Count = 1;
//	}
//
//	time(&endTime);
//	//printf("%d\n", endTime - startTime);
//	if( endTime - startTime > 2)
//	{
//		printf("Write\n");
//		if(m_Count == 1 )
//		{
//			moveFirstPose();
//			m_Count = 2;
//		}
//		else if(m_Count == 2)
//		{
//			moveSecondPose();
//			m_Count = 3;
//		}
//		else if(m_Count == 3)
//		{
//			moveThirdPose();
//			m_Count = 4;
//		}
//		else if(m_Count == 4)
//		{
//			moveFourthPose();
//			m_Count = 5;
//		}
//		else if(m_Count == 5)
//		{
//			moveFifthPose();
//			m_Count = 6;
//		}
//		else if(m_Count == 6)
//		{
//			moveSixthPose();
//			m_Count = 7;
//		}
//		else if(m_Count == 7)
//		{
//			moveSeventhPose();
//			m_Count = 8;
//		}
//		else if(m_Count == 8)
//		{
//			moveEighthPose();
//			m_Count = 9;
//		}
//		else if(m_Count == 9)
//		{
//			moveNinthPose();
//			m_Count = 10;
//		}
//		else if(m_Count == 10)
//		{
//			moveTenthPose();
//			m_Count = 11;
//		}
//		else if(m_Count == 11)
//		{
//			moveEleventhPose();
//			m_Count = 12;
//		}
//		else if(m_Count == 12)
//		{
//			moveTwelfthPose();
//			m_Count = 13;
//		}
//		else if(m_Count == 13)
//		{
//			moveThirteenthPose();
//			m_Count = 14;
//		}
//		else if(m_Count == 14)
//		{
//			moveFourteenthPose();
//			m_Count = 1;
//		}
//
//		time(&startTime);
//	}
}

void Test::moveFirstPose()
{
	for(unsigned int index=0; index < m_RobotInfo.size(); index++)
	{
		int id = -1;
		id = m_RobotInfo[index].m_ID;

		if(id == 1)
			m_RobotInfo[index].m_Value = -50000;
		else if( id == 2)
			m_RobotInfo[index].m_Value = 50000;
		else if(id == 15)
			m_RobotInfo[index].m_Value = 10000;
		else if(id ==16)
			m_RobotInfo[index].m_Value = -10000;
		else if(id == 19)
			m_RobotInfo[index].m_Value = 62750;
		else if(id ==20)
			m_RobotInfo[index].m_Value = -62750;
		else if(id == 23)
			m_RobotInfo[index].m_Value = 10000;
		else if(id == 24)
			m_RobotInfo[index].m_Value = -10000;
		else
			m_RobotInfo[index].m_Value = 0;
	}
}

void Test::moveSecondPose()
{
	for(unsigned int index=0; index < m_RobotInfo.size(); index++)
	{
		int id = -1;
		id = m_RobotInfo[index].m_ID;

		if(id == 1)
			m_RobotInfo[index].m_Value = -125500;
		else if( id == 2)
			m_RobotInfo[index].m_Value = 125500;
		else if( id == 3)
			m_RobotInfo[index].m_Value = -50000;
		else if (id == 4)
			m_RobotInfo[index].m_Value =  50000;
		else if (id == 5)
			m_RobotInfo[index].m_Value = 50000;
		else if (id == 6)
			m_RobotInfo[index].m_Value = -50000;
		else if (id == 7)
			m_RobotInfo[index].m_Value = -62750;
		else if (id == 8)
			m_RobotInfo[index].m_Value = 62750;
	}
}

void Test::moveThirdPose()
{
	for(unsigned int index=0; index < m_RobotInfo.size(); index++)
	{
		int id = -1;
		id = m_RobotInfo[index].m_ID;

		if( id == 3)
			m_RobotInfo[index].m_Value = 0;
		else if (id == 4)
			m_RobotInfo[index].m_Value = 0;
		else if(id == 17)
			m_RobotInfo[index].m_Value = -91375;
		else if( id == 18)
			m_RobotInfo[index].m_Value =  91375;
		else if(id == 19)
			m_RobotInfo[index].m_Value = -120000;
		else if( id == 20)
			m_RobotInfo[index].m_Value = 120000;
		else if(id == 21)
			m_RobotInfo[index].m_Value = 91375;
		else if( id == 22)
			m_RobotInfo[index].m_Value = -91375;
	}
}

void Test::moveFourthPose()
{
	for(unsigned int index=0; index < m_RobotInfo.size(); index++)
	{
		int id = -1;
		id = m_RobotInfo[index].m_ID;

		if(id == 25)
			m_RobotInfo[index].m_Value = 60000;
	}
}

void Test::moveFifthPose()
{
	for(unsigned int index=0; index < m_RobotInfo.size(); index++)
	{
		int id = -1;
		id = m_RobotInfo[index].m_ID;

		if(id == 25)
			m_RobotInfo[index].m_Value = -60000;
	}
}

void Test::moveSixthPose()
{
	for(unsigned int index=0; index < m_RobotInfo.size(); index++)
	{
		int id = -1;
		id = m_RobotInfo[index].m_ID;

		if(id == 25)
			m_RobotInfo[index].m_Value = 0;
	}
}

void Test::moveSeventhPose()
{
	for(unsigned int index=0; index < m_RobotInfo.size(); index++)
	{
		int id = -1;
		id = m_RobotInfo[index].m_ID;

		if(id == 1)
			m_RobotInfo[index].m_Value = -50000;
		else if( id == 2)
			m_RobotInfo[index].m_Value = 50000;
		else if( id == 3)
			m_RobotInfo[index].m_Value = 0;
		else if (id == 4)
			m_RobotInfo[index].m_Value = 0;
		else if (id == 5)
			m_RobotInfo[index].m_Value = 0;
		else if (id == 6)
			m_RobotInfo[index].m_Value = 0;
		else if (id == 7)
			m_RobotInfo[index].m_Value = 150000;
		else if (id == 8)
			m_RobotInfo[index].m_Value = -150000;
	}
}

void Test::moveEighthPose()
{
	for(unsigned int index=0; index < m_RobotInfo.size(); index++)
	{
		int id = -1;
		id = m_RobotInfo[index].m_ID;

		if(id == 25)
			m_RobotInfo[index].m_Value = 60000;
	}
}
void Test::moveNinthPose()
{
	for(unsigned int index=0; index < m_RobotInfo.size(); index++)
	{
		int id = -1;
		id = m_RobotInfo[index].m_ID;

		if(id == 25)
			m_RobotInfo[index].m_Value = -60000;
	}
}
void Test::moveTenthPose()
{
	for(unsigned int index=0; index < m_RobotInfo.size(); index++)
	{
		int id = -1;
		id = m_RobotInfo[index].m_ID;

		if(id == 25)
			m_RobotInfo[index].m_Value = 0;
	}
}

void Test::moveEleventhPose()
{
	for(unsigned int index=0; index < m_RobotInfo.size(); index++)
	{
		int id = -1;
		id = m_RobotInfo[index].m_ID;

		if(id == 1)
			m_RobotInfo[index].m_Value = -50000;
		else if( id == 2)
			m_RobotInfo[index].m_Value = 50000;
		else if( id == 3)
			m_RobotInfo[index].m_Value = 0;
		else if (id == 4)
			m_RobotInfo[index].m_Value = 0;
		else if (id == 5)
			m_RobotInfo[index].m_Value = 0;
		else if (id == 6)
			m_RobotInfo[index].m_Value = 0;
		else if (id == 7)
			m_RobotInfo[index].m_Value = 62750;
		else if (id == 8)
			m_RobotInfo[index].m_Value = -62750;
	}
}

void Test::moveTwelfthPose()
{
	for(unsigned int index=0; index < m_RobotInfo.size(); index++)
	{
		int id = -1;
		id = m_RobotInfo[index].m_ID;

		if(id == 1)
			m_RobotInfo[index].m_Value = -50000;
		else if( id == 2)
			m_RobotInfo[index].m_Value = 50000;
		else if( id == 3)
			m_RobotInfo[index].m_Value = 0;
		else if (id == 4)
			m_RobotInfo[index].m_Value = 0;
		else if (id == 5)
			m_RobotInfo[index].m_Value = 0;
		else if (id == 6)
			m_RobotInfo[index].m_Value = 0;
		else if (id == 7)
			m_RobotInfo[index].m_Value = 150000;
		else if (id == 8)
			m_RobotInfo[index].m_Value = -150000;
	}
}
void Test::moveThirteenthPose()
{
	for(unsigned int index=0; index < m_RobotInfo.size(); index++)
	{
		int id = -1;
		id = m_RobotInfo[index].m_ID;
		if(id == 1)
			m_RobotInfo[index].m_Value = -50000;
		else if( id == 2)
			m_RobotInfo[index].m_Value = 50000;
		else if( id == 3)
			m_RobotInfo[index].m_Value = 0;
		else if (id == 4)
			m_RobotInfo[index].m_Value = 0;
		else if (id == 5)
			m_RobotInfo[index].m_Value = 0;
		else if (id == 6)
			m_RobotInfo[index].m_Value = 0;
		else if (id == 7)
			m_RobotInfo[index].m_Value = 62750;
		else if (id == 8)
			m_RobotInfo[index].m_Value = -62750;
	}
}
void Test::moveFourteenthPose()
{
	for(unsigned int index=0; index < m_RobotInfo.size(); index++)
	{
		int id = -1;
		id = m_RobotInfo[index].m_ID;

		if(id == 1)
			m_RobotInfo[index].m_Value = -50000;
		else if( id == 2)
			m_RobotInfo[index].m_Value = 50000;
		else if( id == 3)
			m_RobotInfo[index].m_Value = 0;
		else if (id == 4)
			m_RobotInfo[index].m_Value = 0;
		else if (id == 5)
			m_RobotInfo[index].m_Value = 0;
		else if (id == 6)
			m_RobotInfo[index].m_Value = 0;
		else if (id == 7)
			m_RobotInfo[index].m_Value = 150000;
		else if (id == 8)
			m_RobotInfo[index].m_Value = -150000;
	}
}
