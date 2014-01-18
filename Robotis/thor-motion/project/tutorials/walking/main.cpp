/*
 * main.cpp
 *
 *  Created on: 2013. 1. 3.
 *      Author: zerom
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <termios.h>
#include <term.h>
#include <fcntl.h>
#include <ncurses.h>
#include <libgen.h>

#include "Thor.h"

#define INI_PATH "config.ini"

using namespace Thor;

int _getch()
{
	struct termios oldt, newt;
	int ch;
	tcgetattr( STDIN_FILENO, &oldt );
	newt = oldt;
	newt.c_lflag &= ~(ICANON | ECHO);
	tcsetattr( STDIN_FILENO, TCSANOW, &newt );
	ch = getchar();
	tcsetattr( STDIN_FILENO, TCSANOW, &oldt );
	return ch;
}

int kbhit(void)
{
	struct termios oldt, newt;
	int ch;
	int oldf;

	tcgetattr(STDIN_FILENO, &oldt);
	newt = oldt;
	newt.c_lflag &= ~(ICANON | ECHO);
	tcsetattr(STDIN_FILENO, TCSANOW, &newt);
	oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
	fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

	ch = getchar();

	tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
	fcntl(STDIN_FILENO, F_SETFL, oldf);

	if(ch != EOF)
	{
		ungetc(ch, stdin);
		return 1;
	}

	return 0;
}

int main(void)
{
	printf( "\n===== Walking Tutorial for Thor =====\n\n");

	MotionManager *manager = MotionManager::GetInstance();

	minIni* ini = new minIni(INI_PATH);

	//Action::GetInstance()->LoadFile("motion_4096.bin");

	if(manager->Initialize() == false)
	{
		printf("Fail to initialize MotionManager.\n");
		return 1;
	}

//	if(manager->InitforTest() == false)
//	{
//		printf("Fail to initialize vel & accel value.\n");
//		return 1;
//	}

	manager->LoadINISettings(ini);
	for(int i = 0; i < 26; i++)
	{
		printf("%d\n", manager->m_Offset[i]);
	}

	manager->AddModule((MotionModule*)Action::GetInstance());

	printf("Press the ENTER key to move init pose!\n");
	getchar();

	manager->StartTimer();

	//    Action::GetInstance()->Start(31);
	//
	//    while(Action::GetInstance()->IsRunning());
	//
	//    Action::GetInstance()->Stop();
	//    manager->StopTimer();
	//    manager->RemoveModule((MotionModule*)Action::GetInstance());


	//    FILE *fp;
	//    fp = fopen("dd1.txt", "w");
	//    for(unsigned int index = 0 ;index < MotionStatus::m_CurrentJoints.size(); index++)
	//    {
	//    	int id = -1;
	//    	id = MotionStatus::m_CurrentJoints[index].m_ID;
	//
	//    	int val = MotionStatus::m_CurrentJoints[index].m_Value - (manager->m_Offset[id-1]);
	//    	printf("id %d\n", id);
	//    	fprintf(fp, "%d : %d , %d,  %d\n", id, MotionStatus::m_CurrentJoints[index].m_Value, val, (int)((val+251000)*2048.0/251000.0));
	//    }
	//    fclose(fp);

	for(int i = 0; i < 36 ; i++)
	{
		MotionStatus::m_EnableList[i].uID = "Walking";
	}

	Ins *ins = new Ins();
	//    if(ins->Connect("ttyACM0", 921600) !=MIP_INTERFACE_OK)
	//    {
	//    	printf("fail to connect\n");
	//    	return 0;
	//    }
	//    printf( "\n===== start initializing ins  =====\n\n");
	//    if(ins->Initialize() != MIP_INTERFACE_OK)
	//    {//    	printf("fail to init\n");
	//    	return 0;
	//    }
	//    printf( "\n===== set enalble data callback =====\n\n");
	//	ins->SetEnableNavDataCallBack();
	//	ins->SetEnableAHRSDataCallBack();
	//	ins->SetEnableGPSDataCallBack();
	//
	//	ins->StartSensingDataUpdate();

	Walking::GetInstance()->BALANCE_ENABLE = false;
	manager->AddModule((MotionModule*)Walking::GetInstance());
	Walking::GetInstance()->LoadINISettings(ini);
	Walking::GetInstance()->P_GAIN = 64;

	printf("Press the ENTER key to begin!\n");
	getchar();

	manager->StartTimer();
	Walking::GetInstance()->X_MOVE_AMPLITUDE = 0;
	Walking::GetInstance()->Start();
//	printf("period : %f\n", Walking::GetInstance()->PERIOD_TIME);
//	printf("period : %f\n", Walking::GetInstance()->HIP_PITCH_OFFSET);
//	printf("period : %f\n", Walking::GetInstance()->ARM_SWING_GAIN);
	while(true)
	{
		if(kbhit())
			if(_getch() == 27)
			{
				Walking::GetInstance()->Stop();
				break;
			}
		//    	printf("%d\n", (int) Walking::GetInstance()->BALANCE_ENABLE );
		//    	printf("FB_GYRO : %f, RL_GYRO : %f, FB_ACCEL : %d, RL_ACCEL: %d\n",
		//    			MotionStatus::FB_GYRO, MotionStatus::RL_GYRO, MotionStatus::FB_ACCEL, MotionStatus::RL_ACCEL);
		usleep(1000);
	}

	while(Walking::GetInstance()->IsRunning())
	{
		usleep(8000);
	}
	manager->StopTimer();

	//ins->StopSensingDataUpdate();
	//ins->Disconnect();
	delete ins;

	FILE *fp;
	fp = fopen("ddd.txt", "w");
	for(unsigned int index = 0 ;index < MotionStatus::m_CurrentJoints.size(); index++)
	{
		int id = -1;
		id = MotionStatus::m_CurrentJoints[index].m_ID;

		int val = MotionStatus::m_CurrentJoints[index].m_Value - (manager->m_Offset[id-1]);
		printf("id %d\n", id);
		//    	fprintf(fp, "%d : %d , %d,  %d\n", id, MotionStatus::m_CurrentJoints[index].m_Value, val, (int)((val+251000.0)*2048.0/251000.0));
		fprintf(fp, "%d : %d,\t%d\n", id, val, (int)((val+251000.0)*2048.0/251000.0));
	}
	fclose(fp);

	return 0;
}
