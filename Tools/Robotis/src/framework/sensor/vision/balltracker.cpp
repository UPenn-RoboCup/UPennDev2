/*
 *   BallTracker.cpp
 *
 *   Author: ROBOTIS
 *
 */

#include <math.h>
#include "motion/modules/head.h"
#include "sensor/vision/camera.h"
#include "sensor/vision/imgprocess.h"
#include "sensor/vision/balltracker.h"

using namespace Thor;


BallTracker::BallTracker() :
        ball_position(Point2D(-1.0, -1.0))
{
	NoBallCount = 0;
}

BallTracker::~BallTracker()
{
}

bool BallTracker::GetTrackerStatus()
{
	if(NoBallCount >= NoBallMaxCount)
		return false;
	else
		return true;
}

void BallTracker::Process(Point2D pos)
{
	if(pos.X < 0 || pos.Y < 0)
	{
		ball_position.X = -1024;
		ball_position.Y = -1024;
		if(NoBallCount < NoBallMaxCount)
		{
			Head::GetInstance()->MoveTracking();
			NoBallCount++;
		}
		else
		{
			Head::GetInstance()->InitTracking();
		}
	}
	else
	{
		NoBallCount = 0;
		Point2D center = Point2D(Camera::WIDTH/2, Camera::HEIGHT/2);
		Point2D offset = pos - center;
		offset *= -1; // Inverse X-axis, Y-axis
		//offset.X *= -1;
		offset.X *= (Camera::VIEW_H_ANGLE / (double)Camera::WIDTH); // pixel per angle
		offset.Y *= (Camera::VIEW_V_ANGLE / (double)Camera::HEIGHT); // pixel per angle
		ball_position = offset;
		Head::GetInstance()->MoveTracking(ball_position);
	}
}
