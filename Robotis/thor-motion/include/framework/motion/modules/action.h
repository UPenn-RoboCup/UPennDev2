/*
 * Action.h
 *
 *  Created on: 2013. 1. 17.
 *      Author: hjsong
 */

#ifndef ACTION_H_
#define ACTION_H_

#include <stdio.h>
#include "motion/motionmodule.h"
#include "motion/motionstatus.h"

namespace Thor
{
class Action : public MotionModule
{

public:
	enum
	{
		MAXNUM_PAGE = 256,
		MAXNUM_STEP = 7,
		MAXNUM_NAME = 13
	};

	enum
	{
		SPEED_BASE_SCHEDULE = 0,
		TIME_BASE_SCHEDULE = 0x0a
	};

	enum
	{
		INVALID_BIT_MASK	= 0x4000,
		TORQUE_OFF_BIT_MASK	= 0x2000
	};

	typedef struct // Header Structure (total 64unsigned char)
	{
		unsigned char name[MAXNUM_NAME+1]; // Name             0~13
		unsigned char reserved1;        // Reserved1        14
		unsigned char repeat;           // Repeat count     15
		unsigned char schedule;         // schedule         16
		unsigned char reserved2[3];     // reserved2        17~19
		unsigned char stepnum;          // Number of step   20
		unsigned char reserved3;        // reserved3        21
		unsigned char speed;            // Speed            22
		unsigned char reserved4;        // reserved4        23
		unsigned char accel;            // Acceleration time 24
		unsigned char next;             // Link to next     25
		unsigned char exit;             // Link to exit     26
		unsigned char reserved5[4];     // reserved5        27~30
		unsigned char checksum;         // checksum         31
		unsigned int  pgain[38];        // CW/CCW compliance slope  32~62
		unsigned char reserved6;        // reserved6        63
	} PAGEHEADER;

	typedef struct // Step Structure (total 64unsigned char)
	{
		unsigned short position[38];            // Joint position   0~72
		unsigned char pause;            // Pause time       73
		unsigned char time;             // Time             74
	} STEP;

	typedef struct // Page Structure (total 512unsigned char)
	{
		PAGEHEADER header;          // Page header  0~64
		STEP step[MAXNUM_STEP];		// Page step    65~511
	} PAGE;

private:
	static Action* m_UniqueInstance;
	FILE* m_ActionFile;
	PAGE m_PlayPage;
	PAGE m_NextPlayPage;
	STEP m_CurrentStep;

	int m_IndexPlayingPage;
	bool m_FirstDrivingStart;
	int m_PageStepCount;
	bool m_Playing;
	bool m_StopPlaying;
	bool m_PlayingFinished;

	Action();

	bool VerifyChecksum( PAGE *pPage );
	void SetChecksum( PAGE *pPage );

public:
	bool DEBUG_PRINT;

	virtual ~Action();

	static Action* GetInstance() { return m_UniqueInstance; }

	void Initialize();
	void Process();
	bool LoadFile(char* filename);
	bool CreateFile(char* filename);

	bool Start(int iPage);
	bool Start(char* namePage);
	bool Start(int index, PAGE *pPage);

	bool Start(int iPage, EnableList *enable_List);
	bool Start(char* namePage, EnableList *enable_List);
	bool Start(int index, PAGE *pPage, EnableList *enable_List);

	void Stop();
	void Brake();
	bool IsRunning();
	bool IsRunning(int *iPage, int *iStep);
	bool LoadPage(int index, PAGE *pPage);
	bool SavePage(int index, PAGE *pPage);
	void ResetPage(PAGE *pPage);
};

}


#endif /* ACTION_H_ */
