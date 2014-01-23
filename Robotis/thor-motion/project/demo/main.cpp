/*
 * main.cpp
 *
 *  Created on: 2013. 2. 14.
 *      Author: hjsong
 */

#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <libgen.h>
#include "streamer/mjpg_streamer.h"
#include "Thor.h"

#define INI_FILE_PATH "config.ini"

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

void PrintErrorCode(int ErrorCode)
{
	printf("ErrorCode : %d (0x%X)\n", ErrorCode, ErrorCode);
    if(ErrorCode & ERRBIT_VOLTAGE)
        printf("Input voltage error!\n");

    if(ErrorCode & ERRBIT_ANGLE)
        printf("Angle limit error!\n");

    if(ErrorCode & ERRBIT_OVERHEAT)
        printf("Overheat error!\n");

    if(ErrorCode & ERRBIT_RANGE)
        printf("Out of range error!\n");

    if(ErrorCode & ERRBIT_CHECKSUM)
        printf("Checksum error!\n");

    if(ErrorCode & ERRBIT_OVERLOAD)
        printf("Overload error!\n");

    if(ErrorCode & ERRBIT_INSTRUCTION)
        printf("Instruction code error!\n");
}

int main(void)
{
	MotionManager *manager = MotionManager::GetInstance();

	minIni* ini = new minIni(INI_FILE_PATH);

	Walking::GetInstance()->LoadINISettings(ini);
	Walking::GetInstance()->BALANCE_ENABLE = true;
//	Walking::GetInstance()->FB_ALL_BALANCE_ENABLE =true;
//	Walking::GetInstance()->RL_ALL_BALANCE_ENABLE = true;

	Walking::GetInstance()->P_GAIN = 64;
	Action::GetInstance()->LoadFile("motion_4096.bin");

	if(manager->Initialize() == false)
	{
		printf("Fail to initialize MotionManager.\n");
		return 1;
	}

	MotionStatus::m_CurrentJoints[30].m_Value = 2048;
	MotionStatus::m_CurrentJoints[31].m_Value = 2048;
	MotionStatus::m_CurrentJoints[32].m_Value = 2048;
	MotionStatus::m_CurrentJoints[33].m_Value = 2048;
	MotionStatus::m_CurrentJoints[34].m_Value = 2048;
	MotionStatus::m_CurrentJoints[35].m_Value = 2048;

	for(unsigned int index = 0; index < MotionStatus::m_CurrentJoints.size(); index++)
	{
		int id = MotionStatus::m_CurrentJoints[index].m_ID;

		int err, val;
		if( id < 29 )
		{
			MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteDWord(id, PRO54::P_GOAL_ACCELATION_LL, 0, &err);
			printf("id : %d  ", id);
			PrintErrorCode(err);
		}
		usleep(1000);
	}



	Camera::GetInstance()->Initialize(0);

	mjpg_streamer* streamer = new mjpg_streamer(Camera::WIDTH, Camera::HEIGHT);

	manager->LoadINISettings(ini);
	manager->AddModule((MotionModule*)Action::GetInstance());
	manager->AddModule((MotionModule*)Walking::GetInstance());
	Walking::GetInstance()->LoadINISettings(ini);
	Walking::GetInstance()->BALANCE_ENABLE = true;
	Walking::GetInstance()->P_GAIN = 64;

	Ins *ins = new Ins();
	if(ins->Connect("ttyACM0", 921600) !=MIP_INTERFACE_OK)
	{
		printf("fail to connect ins\n");
		return 0;
	}
	printf( "\n===== start initializing ins  =====\n\n");
	if(ins->Initialize() != MIP_INTERFACE_OK)
	{
		printf("fail to init ins\n");
		return 0;
	}

	printf( "\n===== set enalble ins data callback =====\n\n");
	ins->SetEnableAHRSDataCallBack();
	ins->StartSensingDataUpdate();

	printf("Press the ENTER key to move init pose!\n");
	getchar();
	manager->StartTimer();
	Action::GetInstance()->Start(19);
	while(Action::GetInstance()->IsRunning())
		usleep(8000);

	MotionManager::GetInstance()->StopTimer();

	usleep(8000);
	for(unsigned int index = 0; index < MotionStatus::m_CurrentJoints.size(); index++)
	{
		usleep(1);
		int id = MotionStatus::m_CurrentJoints[index].m_ID;

//		if(id > 12 && id <25)
//			MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteWord(id, PRO54::P_VELOCITY_I_GAIN_L, 0, 0);
	}

	MotionManager::GetInstance()->StartTimer();
//	Walking::GetInstance()->FB_ALL_BALANCE_ENABLE =true;
//	Walking::GetInstance()->RL_ALL_BALANCE_ENABLE = true;
	for(int i =0; i < 14; i++)
	{
		MotionStatus::m_EnableList[i].uID = "Action";
	}

	MotionStatus::m_EnableList[14].uID = "Walking";
	MotionStatus::m_EnableList[15].uID = "Walking";
	MotionStatus::m_EnableList[16].uID = "Walking";
	MotionStatus::m_EnableList[17].uID = "Walking";
	MotionStatus::m_EnableList[18].uID = "Walking";
	MotionStatus::m_EnableList[19].uID = "Walking";
	MotionStatus::m_EnableList[20].uID = "Walking";
	MotionStatus::m_EnableList[21].uID = "Walking";
	MotionStatus::m_EnableList[22].uID = "Walking";
	MotionStatus::m_EnableList[23].uID = "Walking";
	MotionStatus::m_EnableList[24].uID = "Walking";
	MotionStatus::m_EnableList[25].uID = "Walking";


	printf("Start Demonstration!!\n");

	while(true)
	{
		if(kbhit())
		{
			char temp = _getch();
			if(temp == 27)
				break;
			else if(temp == 'l' || temp == 'L')
			{
				Walking::GetInstance()->LoadINISettings(ini);
				manager->LoadINISettings(ini);
				printf("\n load ini!!\n");
			}
			else if(temp == 'g')
			{
				MotionStatus::m_CurrentJoints[30].m_Value = 1955;
				MotionStatus::m_CurrentJoints[31].m_Value = 2147;

				MotionStatus::m_CurrentJoints[32].m_Value = 2158;
				MotionStatus::m_CurrentJoints[33].m_Value = 1971;

				MotionStatus::m_CurrentJoints[34].m_Value = 1940;
				MotionStatus::m_CurrentJoints[35].m_Value = 2132;
				printf("\n grab!!\n");
			}
			else if(temp == 'r')
			{
				MotionStatus::m_CurrentJoints[30].m_Value = 2048;
				MotionStatus::m_CurrentJoints[31].m_Value = 2048;
				MotionStatus::m_CurrentJoints[32].m_Value = 2048;
				MotionStatus::m_CurrentJoints[33].m_Value = 2048;
				MotionStatus::m_CurrentJoints[34].m_Value = 2048;
				MotionStatus::m_CurrentJoints[35].m_Value = 2048;
				printf("\n release!!\n");
			}
			else if(temp == 'h')
			{
				MotionStatus::m_CurrentJoints[30].m_Value = 2265;
				MotionStatus::m_CurrentJoints[32].m_Value = 1730;
				MotionStatus::m_CurrentJoints[34].m_Value = 2369;

				MotionStatus::m_CurrentJoints[31].m_Value = 2048;
				MotionStatus::m_CurrentJoints[33].m_Value = 2048;
				MotionStatus::m_CurrentJoints[35].m_Value = 2048;
				printf("\n release!!\n");
			}
			else if(temp == 's')
			{
				Action::GetInstance()->Stop();
			}
		}

		Camera::GetInstance()->CaptureFrame();
		streamer->send_image(Camera::GetInstance()->fbuffer->m_YUVFrame);
		usleep(10000);
	}

	Walking::GetInstance()->Stop();
	while(Walking::GetInstance()->IsRunning())
		usleep(1000);


	Action::GetInstance()->Stop();
	while(Action::GetInstance()->IsRunning())
		usleep(1000);

	ins->StopSensingDataUpdate();
	manager->StopTimer();

	delete streamer;
	delete ins;

	return 0;
}
