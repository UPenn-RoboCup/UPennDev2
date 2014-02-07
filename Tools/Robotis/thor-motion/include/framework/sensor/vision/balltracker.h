/*
 *   BallTracker.h
 *
 *   Author: ROBOTIS
 *
 */

#ifndef _BALL_TRACKER_H_
#define _BALL_TRACKER_H_

#include <string.h>

#include "math/point.h"
#include "framework/minIni.h"

namespace Thor
{
	class BallTracker
	{
	private:
		int NoBallCount;
		static const int NoBallMaxCount = 15;

	public:
        Point2D     ball_position;

		BallTracker();
		~BallTracker();

		bool GetTrackerStatus();
		void Process(Point2D pos);
	};
}

#endif
