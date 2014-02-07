/*
 * motiontimer.cpp
 *
 *  Created on: 2013. 1. 11.
 *      Author: zerom
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "motiontimer.h"

using namespace Thor;

MotionTimer::MotionTimer() :
		ThreadID(0),
		IsTimerRunning(false),
		IsTimerStop(false)
{
}

MotionTimer::~MotionTimer()
{
	this->Stop();
}

bool MotionTimer::IsRunning()
{
	return this->IsTimerRunning;
}

void* MotionTimer::TimerProc(void* param)
{
	MotionTimer* timer = (MotionTimer*)param;
	static struct timespec next_time;
	clock_gettime(CLOCK_MONOTONIC, &next_time);

	while(!timer->IsTimerStop)
	{
        next_time.tv_sec += (next_time.tv_nsec + TIME_UNIT * 1000000) / 1000000000;
        next_time.tv_nsec = (next_time.tv_nsec + TIME_UNIT * 1000000) % 1000000000;

        //TODO: Call process function
        //if(timer->m_Manager != NULL)
        //    timer->m_Manager->Process();

        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, 0);
	}

	pthread_exit(0);
	return 0;
}

void MotionTimer::Start()
{
	int error = 0;
	struct sched_param param;
	pthread_attr_t attr;

	pthread_attr_init(&attr);

	error = pthread_attr_setschedpolicy(&attr, SCHED_RR);
	if(error != 0)
		fprintf(stderr, "[MotionTimer] pthread_attr_setschedpolicy Error = %d \n", error);
	error = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
	if(error != 0)
		fprintf(stderr, "[MotionTimer] pthread_attr_setinheritsched Error = %d \n", error);

	memset(&param, 0, sizeof(param));
	param.sched_priority = 31; // RealTime
	error = pthread_attr_setschedparam(&attr, &param);
	if(error != 0)
		fprintf(stderr, "[MotionTimer] pthread_attr_setschedparam Error = %d \n", error);

	if(pthread_create(&this->ThreadID, &attr, this->TimerProc, this) != 0)
		exit(-1);

	this->IsTimerRunning = true;
}

void MotionTimer::Stop()
{
	if(this->IsTimerRunning == true)
	{
		this->IsTimerStop = true;
		// wait for the thread to end
		if(pthread_join(this->ThreadID, 0) != 0)
			exit(-1);
		this->IsTimerStop = false;
		this->IsTimerRunning = false;
	}
}
