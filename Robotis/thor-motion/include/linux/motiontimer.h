/*
 * motiontimer.h
 *
 *  Created on: 2013. 1. 11.
 *      Author: zerom
 */

#ifndef MOTIONTIMER_H_
#define MOTIONTIMER_H_

#include <time.h>
#include <pthread.h>

namespace Thor
{

class MotionTimer
{
private:
	static const int TIME_UNIT	= 8;

	pthread_t	ThreadID;
	bool		IsTimerRunning;
	bool		IsTimerStop;

protected:
	static void *TimerProc(void *param);

public:
	MotionTimer();
	virtual ~MotionTimer();

	void Start();
	void Stop();
	bool IsRunning();
};

} /* namespace Thor */
#endif /* MOTIONTIMER_H_ */
