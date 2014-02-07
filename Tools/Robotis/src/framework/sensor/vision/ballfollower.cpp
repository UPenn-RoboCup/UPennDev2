/*
 *   BallFollower.cpp
 *
 *   Author: ROBOTIS
 *
 */

#include <stdio.h>
#include "sensor/vision/imgprocess.h"
#include "motion/modules/action.h"
#include "motion/modules/walking.h"
#include "motion/modules/head.h"
#include "motion/motionstatus.h"
#include "motion/PRO42.h"
#include "sensor/vision/ballfollower.h"

using namespace Thor;


BallFollower::BallFollower()
{
	m_NoBallMaxCount = 10;
	m_NoBallCount = m_NoBallMaxCount;
	m_KickBallMaxCount = 10;
	m_KickBallCount = 0;

	m_KickTopAngle = -5.0;
	m_KickRightAngle = -30.0;
	m_KickLeftAngle = 30.0;

	m_FollowMaxFBStep = 50.0;
    m_FollowMinFBStep = 5.0;
	m_FollowMaxRLTurn = 15.0;
	m_FitFBStep = 3.0;
	m_FitMaxRLTurn = 35.0;
	m_UnitFBStep = 5.0;
	m_UnitRLTurn = 1.0;

	m_GoalFBStep = 0;
	m_GoalRLTurn = 0;
	m_FBStep = 0;
	m_RLTurn = 0;
	DEBUG_PRINT = false;
	KickBall = 0;

	m_Pan_JointIndex = 0;
	m_Tilt_JointIndex = 0;

	m_FirstDriving = true;
}

BallFollower::~BallFollower()
{
}

PRO42 pro42;

void BallFollower::Process(Point2D ball_pos)
{
	if(DEBUG_PRINT == true)
		fprintf(stderr, "\r                                                                               \r");

	if(m_FirstDriving)
	{
		for(unsigned int jointIndex = 0 ; jointIndex < MotionStatus::m_CurrentJoints.size() ; jointIndex++)
			if(MotionStatus::m_CurrentJoints[jointIndex].m_ID == 29)
			{
				m_Pan_JointIndex = jointIndex;
			}
			else if(MotionStatus::m_CurrentJoints[jointIndex].m_ID == 30)
			{
				m_Tilt_JointIndex = jointIndex;
			}
			else if( (m_Pan_JointIndex != 0) && (m_Tilt_JointIndex != 0) )
				break;

		m_FirstDriving = false;
	}

    if(ball_pos.X == -1024.0 || ball_pos.Y == -1024.0)
    {
		KickBall = 0;

		if(m_NoBallCount > m_NoBallMaxCount)
		{
			// can not find a ball
			m_GoalFBStep = 0;
			m_GoalRLTurn = 0;
			Head::GetInstance()->MoveToHome();

			if(DEBUG_PRINT == true)
				fprintf(stderr, "[NO BALL]");
		}
		else
		{
			m_NoBallCount++;
			if(DEBUG_PRINT == true)
				fprintf(stderr, "[NO BALL COUNTING(%d/%d)]", m_NoBallCount, m_NoBallMaxCount);
		}
    }
    else
    {
		m_NoBallCount = 0;		

		double pan = MotionStatus::m_CurrentJoints[m_Pan_JointIndex].m_DXLInfo->Value2Angle(MotionStatus::m_CurrentJoints[m_Pan_JointIndex].m_Value);
		double pan_range = Head::GetInstance()->GetLeftLimitAngle();
		double pan_percent = pan / pan_range;

		double tilt = MotionStatus::m_CurrentJoints[m_Tilt_JointIndex].m_DXLInfo->Value2Angle(MotionStatus::m_CurrentJoints[m_Tilt_JointIndex].m_Value);
		double tilt_max = Head::GetInstance()->GetTopLimitAngle();
		double tilt_min = Head::GetInstance()->GetBottomLimitAngle();
		double tilt_range = Head::GetInstance()->GetTopLimitAngle() - Head::GetInstance()->GetBottomLimitAngle();
		double tilt_percent = (tilt_max - tilt) / tilt_range;
		if(tilt_percent < 0)
			tilt_percent = -tilt_percent;

		if(pan > m_KickRightAngle && pan < m_KickLeftAngle)
		{
			if(  tilt  >=  tilt_max - 10.0*pro42.MAX_ANGLE/((double)pro42.MAX_VALUE)  )
			{
				if(ball_pos.Y < m_KickTopAngle)
				{
					m_GoalFBStep = 0;
					m_GoalRLTurn = 0;

					if(m_KickBallCount >= m_KickBallMaxCount)
					{
						m_FBStep = 0;
						m_RLTurn = 0;
						if(DEBUG_PRINT == true)
							fprintf(stderr, "[KICK]");

						if(pan > 0)
						{
							KickBall = 1; // Left
							if(DEBUG_PRINT == true)
								fprintf(stderr, " Left");
						}
						else
						{
							KickBall = -1; // Right
							if(DEBUG_PRINT == true)
								fprintf(stderr, " Right");
						}
					}
					else
					{
						KickBall = 0;
						if(DEBUG_PRINT == true)
							fprintf(stderr, "[KICK COUNTING(%d/%d)]", m_KickBallCount, m_KickBallMaxCount);
					}
				}
				else
				{
					m_KickBallCount = 0;
					KickBall = 0;
					m_GoalFBStep = m_FitFBStep;
					m_GoalRLTurn = m_FitMaxRLTurn * pan_percent;
					if(DEBUG_PRINT == true)
						fprintf(stderr, "[FIT(P:%.2f T:%.2f>%.2f)]", pan, ball_pos.Y, m_KickTopAngle);
				}
			}
			else
			{
				m_KickBallCount = 0;
				KickBall = 0;
				m_GoalFBStep = m_FollowMaxFBStep * tilt_percent;
				if(m_GoalFBStep < m_FollowMinFBStep)
				    m_GoalFBStep = m_FollowMinFBStep;
				m_GoalRLTurn = m_FollowMaxRLTurn * pan_percent;
				if(DEBUG_PRINT == true)
					fprintf(stderr, "[FOLLOW(P:%.2f T:%.2f>%.2f]", pan, tilt, tilt_min);
			}
		}
		else
		{
			m_KickBallCount = 0;
			KickBall = 0;
			m_GoalFBStep = 0;
			m_GoalRLTurn = m_FollowMaxRLTurn * pan_percent;
			if(DEBUG_PRINT == true)
				fprintf(stderr, "[FOLLOW(P:%.2f T:%.2f>%.2f]", pan, tilt, tilt_min);
		}
	}


	if(m_GoalFBStep == 0 && m_GoalRLTurn == 0 && m_FBStep == 0 && m_RLTurn == 0)
	{
		if(Walking::GetInstance()->IsRunning() == true)
			Walking::GetInstance()->Stop();
		else
		{
			if(m_KickBallCount < m_KickBallMaxCount)
				m_KickBallCount++;
		}

		if(DEBUG_PRINT == true)
			fprintf(stderr, " STOP");
	}
	else
	{
		if(DEBUG_PRINT == true)
			fprintf(stderr, " START");

		if(Walking::GetInstance()->IsRunning() == false)
		{
			m_FBStep = 0;
			m_RLTurn = 0;
			m_KickBallCount = 0;
			KickBall = 0;
			Walking::GetInstance()->X_MOVE_AMPLITUDE = m_FBStep*0.5;
			Walking::GetInstance()->A_MOVE_AMPLITUDE = m_RLTurn*0.5;
			Walking::GetInstance()->Start();			
		}
		else
		{
			if(m_FBStep < m_GoalFBStep)
				m_FBStep += m_UnitFBStep;
			else if(m_FBStep > m_GoalFBStep)
				m_FBStep = m_GoalFBStep;
			Walking::GetInstance()->X_MOVE_AMPLITUDE = m_FBStep*1.0;

			if(m_RLTurn < m_GoalRLTurn)
				m_RLTurn += m_UnitRLTurn;
			else if(m_RLTurn > m_GoalRLTurn)
				m_RLTurn -= m_UnitRLTurn;
			Walking::GetInstance()->A_MOVE_AMPLITUDE = m_RLTurn*1.0;

			if(DEBUG_PRINT == true)
				fprintf(stderr, " (FB:%.1f RL:%.1f)", m_FBStep, m_RLTurn);
		}
	}

}
