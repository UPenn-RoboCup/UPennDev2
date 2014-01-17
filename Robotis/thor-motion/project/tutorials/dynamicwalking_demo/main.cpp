/*
 * main.cpp
 *
 *  Created on: 2013. 1. 3.
 *      Author: zerom
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <getopt.h>
#include <string.h>
#include <termios.h>




#include <term.h>
#include <fcntl.h>
#include <ncurses.h>


#include "motion/modules/action.h"
#include "motion/modules/walking.h"
#include "motion/modules/walking130610.h"
#include "motion/modules/dynamic_walking131012.h"
#include "motion/modules/head.h"
#include "sensor/vision/image.h"
#include "sensor/vision/imgprocess.h"
#include "sensor/vision/balltracker.h"
#include "sensor/vision/ballfollower.h"
#include "sensor/vision/colorfinder.h"
#include "sensor/vision/camera.h"
#include "motion/PRO54.h"
#include "motion/PRO42.h"
#include "motion/motionstatus.h"
#include "motion/motionmanager.h"
#include "motion/dxl_comm.h"
#include "sensor/ins/ins.h"
#include "sensor/lidar/lidar.h"

#include "framework/bulkread.h"

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

void Usage(char *progname)
{
	fprintf(stderr, "-----------------------------------------------------------------------\n");
	fprintf(stderr, "Usage: %s\n" \
			" [-h | --help]........: display this help\n" \
			" [-d | --device]......: port to open                     (/dev/ttyUSB0)\n" \
			, progname);
	fprintf(stderr, "-----------------------------------------------------------------------\n");
}

void Help()
{
	fprintf(stderr, "\n");
	fprintf(stderr, "COMMAND: \n\n" \
			" a                         : forward walking\n"
			" b                         : block climbing \n" \
			" c                         : walking down the block \n" \
			" d                         : ??? \n" \
			" exit                      : Exit this program \n" \
	);
	fprintf(stderr, "\n");
}

// Print error bit of status packet
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

int main(int argc, char *argv[])
{
	fprintf(stderr, "\n***********************************************************************\n");
	fprintf(stderr,   "*                     Demo for Dynamic Walking                        *\n");
	fprintf(stderr,   "***********************************************************************\n\n");



	if(MotionManager::GetInstance()->Initialize() == false)
	{
		printf("Fail to initialize Motion Manager!\n");
		return 0;
	}

	for(unsigned int index = 0; index < MotionStatus::m_CurrentJoints.size(); index++)
	{
		int id = MotionStatus::m_CurrentJoints[index].m_ID;

		int err, val;
		MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteByte(id, PRO54::P_TORQUE_ENABLE, 1, &err);
		MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteDWord(id, PRO54::P_GOAL_ACCELATION_LL, 4, &err);
		MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteDWord(id, PRO54::P_GOAL_VELOCITY_LL, 2000, &err);
		printf("id : %d  ", id);
		PrintErrorCode(err);
		usleep(1000);
	}


	Ins *ins = new Ins();
	if(ins->Connect("ttyACM0", 921600) !=MIP_INTERFACE_OK)
	{
		printf("fail to connect\n");
		return 0;
	}
	printf( "\n===== start initializing ins  =====\n\n");
	if(ins->Initialize() != MIP_INTERFACE_OK)
	{
		printf("fail to init\n");
		return 0;
	}
	printf( "\n===== set enalble data callback =====\n\n");

	if(ins->SetEnableAHRSDataCallBack() != MIP_INTERFACE_OK)
	{
		printf("fail to init\n");
		return 0;
	}
	ins->StartSensingDataUpdate();


	minIni* ini = new minIni("config.ini");
	MotionManager::GetInstance()->LoadINISettings(ini);



	for(unsigned int index = 0; index < MotionStatus::m_CurrentJoints.size(); index++)
	{
		int id = MotionStatus::m_CurrentJoints[index].m_ID;

		int err = 0;
		if( id == 1)
			MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteDWord(id, PRO54::P_GOAL_POSITION_LL, -62750, &err);
		else if(id == 2)
			MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteDWord(id, PRO54::P_GOAL_POSITION_LL, 62750, &err);
		else if(id == 3)
			MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteDWord(id, PRO54::P_GOAL_POSITION_LL, -111910, &err);
		else if(id == 4)
			MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteDWord(id, PRO54::P_GOAL_POSITION_LL, 111910, &err);
		else if(id == 5)
			MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteDWord(id, PRO54::P_GOAL_POSITION_LL, 125500, &err);
		else if(id == 6)
			MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteDWord(id, PRO54::P_GOAL_POSITION_LL, -125500, &err);
		else if(id == 7)
			MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteDWord(id, PRO54::P_GOAL_POSITION_LL, 62750, &err);
		else if(id == 8)
			MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteDWord(id, PRO54::P_GOAL_POSITION_LL, -62750, &err);
		else if(id == 9)
			MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteDWord(id, PRO54::P_GOAL_POSITION_LL, -75000, &err);
		else if(id == 10)
			MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteDWord(id, PRO54::P_GOAL_POSITION_LL,  75000, &err);
		else if(id == 11)
			MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteDWord(id, PRO54::P_GOAL_POSITION_LL,  0, &err);
		else if(id == 12)
			MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteDWord(id, PRO54::P_GOAL_POSITION_LL,  0, &err);
		else if(id == 13)
			MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteDWord(id, PRO54::P_GOAL_POSITION_LL,  0, &err);
		else if(id == 14)
			MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteDWord(id, PRO54::P_GOAL_POSITION_LL,  0, &err);

		else if(id == 15)
			MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteDWord(id, PRO54::P_GOAL_POSITION_LL,  0 + MotionManager::GetInstance()->m_Offset[14], &err);
		else if(id == 17)
			MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteDWord(id, PRO54::P_GOAL_POSITION_LL,  2629 + MotionManager::GetInstance()->m_Offset[16], &err);
		else if(id == 19)
			MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteDWord(id, PRO54::P_GOAL_POSITION_LL,  49364+ MotionManager::GetInstance()->m_Offset[18], &err);
		else if(id == 21)
			MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteDWord(id, PRO54::P_GOAL_POSITION_LL,  -17477 - 1700 + MotionManager::GetInstance()->m_Offset[20], &err);
		else if(id == 23)
			MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteDWord(id, PRO54::P_GOAL_POSITION_LL,  -41922 + 1029 + MotionManager::GetInstance()->m_Offset[22], &err);
		else if(id == 25)
			MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteDWord(id, PRO54::P_GOAL_POSITION_LL,  2296 + 345 + MotionManager::GetInstance()->m_Offset[24], &err);

		else if(id == 16)
			MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteDWord(id, PRO54::P_GOAL_POSITION_LL,  0  + MotionManager::GetInstance()->m_Offset[15], &err);
		else if(id == 18)
			MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteDWord(id, PRO54::P_GOAL_POSITION_LL,  -4729 + 2150 + MotionManager::GetInstance()->m_Offset[17], &err);
		else if(id == 20)
			MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteDWord(id, PRO54::P_GOAL_POSITION_LL,  -49566+200 + MotionManager::GetInstance()->m_Offset[19], &err);
		else if(id == 22)
			MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteDWord(id, PRO54::P_GOAL_POSITION_LL,  17980+1200 + MotionManager::GetInstance()->m_Offset[21], &err);
		else if(id == 24)
			MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteDWord(id, PRO54::P_GOAL_POSITION_LL,  42110-1215 + MotionManager::GetInstance()->m_Offset[23], &err);
		else if(id == 26)
			MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteDWord(id, PRO54::P_GOAL_POSITION_LL,  -3061+496 + MotionManager::GetInstance()->m_Offset[25], &err);

		else if(id == 27)
			MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteDWord(id, PRO54::P_GOAL_POSITION_LL,  0, &err);
		else if(id == 28)
			MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteDWord(id, PRO54::P_GOAL_POSITION_LL,  0, &err);

		usleep(1000);
	}

	usleep(7000000);
	printf("Press the any button. After the moving stop!\n");
	_getch();



	MotionManager::GetInstance()->Reinitialize();
	DynamicWalking131012::GetInstance()->Initialize();
	DynamicWalking131012::GetInstance()->BALANCE_ENABLE = true;
	DynamicWalking131012::GetInstance()->BALANCE_KNEE_GAIN = 0.3*0.5*2.0;
	DynamicWalking131012::GetInstance()->BALANCE_ANKLE_PITCH_GAIN = 0.9*0.5*2.0;
	DynamicWalking131012::GetInstance()->BALANCE_HIP_ROLL_GAIN = 0.5*1.0;
	DynamicWalking131012::GetInstance()->BALANCE_ANKLE_ROLL_GAIN = 1.0*1.0;

	DynamicWalking131012::GetInstance()->HIP_PITCH_OFFSET = 8.0;//8.0

	DynamicWalking131012::GetInstance()->P_GAIN = 64;
	DynamicWalking131012::GetInstance()->I_GAIN = 0;
	DynamicWalking131012::GetInstance()->D_GAIN = 0;

	MotionManager::GetInstance()->AddModule((MotionModule*)DynamicWalking131012::GetInstance());




	for(unsigned int index = 0; index < MotionStatus::m_CurrentJoints.size(); index++)
	{
		int id = MotionStatus::m_CurrentJoints[index].m_ID;

		int err, val;
		MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteDWord(id, PRO54::P_GOAL_ACCELATION_LL, 0, &err);
		MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteDWord(id, PRO54::P_GOAL_VELOCITY_LL, 0, &err);
		printf("id : %d  ", id);
		PrintErrorCode(err);
		usleep(1000);
	}

	char input[128];
	char cmd[80];
	char param[20][30];
	int num_param;
	char* token;
	while(1)
	{
		printf("[CMD] ");
		gets(input);
		fflush(stdin);

		if(strlen(input) == 0)
			continue;

		token = strtok(input, " ");
		if(token == 0)
			continue;

		strcpy(cmd, token);
		token = strtok(0, " ");
		num_param = 0;
		while(token != 0)
		{
			strcpy(param[num_param++], token);
			token = strtok(0, " ");
		}

		if(strcmp(cmd, "a") == 0)
		{

			printf("Start Forward Walking Demo!\n");
			_getch();

			DynamicWalking131012::GetInstance()->SetFileName("ForwardWalking.txt");
			DynamicWalking131012::GetInstance()->ReInitialize();

			for(unsigned int jointIndex = 0; jointIndex < 35; jointIndex++)
			{
				int id = jointIndex;

				//not be fixed code
				if(id >= 15 && id <=26)
				{
					MotionStatus::m_EnableList[id-1].uID = "DynamicWalking";
				}
				if(id == 1 || id ==2 || id == 7|| id ==8)
					MotionStatus::m_EnableList[id-1].uID = "DynamicWalking";
			}

			MotionManager::GetInstance()->StartTimer();

			printf("Playing.... If you want to stop this demo, Press the ESC Button!\n");
			while(true)
			{
				if(kbhit())
					if(_getch() == 27)
					{
						break;
					}
			}
			printf("Demo terminated by user!\n");
			MotionManager::GetInstance()->StopTimer();
		}
		else if(strcmp(cmd, "b") == 0)
		{
			printf("Start Climbing Block Demo!\n");
			_getch();

			DynamicWalking131012::GetInstance()->SetFileName("ClimbBlock.txt");
			DynamicWalking131012::GetInstance()->ReInitialize();

			for(unsigned int jointIndex = 0; jointIndex < 35; jointIndex++)
			{
				int id = jointIndex;

				//not be fixed code
				if(id >= 15 && id <=26)
				{
					MotionStatus::m_EnableList[id-1].uID = "DynamicWalking";
				}
				if(id == 1 || id ==2 || id == 7|| id ==8)
					MotionStatus::m_EnableList[id-1].uID = "DynamicWalking";
			}

			MotionManager::GetInstance()->StartTimer();

			printf("Playing.... If you want to stop this demo, Press the ESC Button!\n");
			while(true)
			{
				if(kbhit())
					if(_getch() == 27)
					{
						break;
					}
			}
			printf("Demo terminated by user!\n");
			MotionManager::GetInstance()->StopTimer();

		}
		else if(strcmp(cmd, "c")==0)
		{
			printf("Start Walking Down Block Demo!\n");
			_getch();

			DynamicWalking131012::GetInstance()->SetFileName("DownBlock.txt");
			DynamicWalking131012::GetInstance()->ReInitialize();

			for(unsigned int jointIndex = 0; jointIndex < 35; jointIndex++)
			{
				int id = jointIndex;

				//not be fixed code
				if(id >= 15 && id <=26)
				{
					MotionStatus::m_EnableList[id-1].uID = "DynamicWalking";
				}
				if(id == 1 || id ==2 || id == 7|| id ==8)
					MotionStatus::m_EnableList[id-1].uID = "DynamicWalking";
			}

			MotionManager::GetInstance()->StartTimer();

			printf("Playing.... If you want to stop this demo, Press the ESC Button!\n");
			while(true)
			{
				if(kbhit())
					if(_getch() == 27)
					{
						break;
					}
			}
			printf("Demo terminated by user!\n");
			MotionManager::GetInstance()->StopTimer();
		}
		else if(strcmp(cmd, "d") == 0)
		{
			printf("Start ????? Demo!\n");
			_getch();

			DynamicWalking131012::GetInstance()->SetFileName("Angles.txt");
			DynamicWalking131012::GetInstance()->ReInitialize();

			for(unsigned int jointIndex = 0; jointIndex < 35; jointIndex++)
			{
				int id = jointIndex;

				//not be fixed code
				if(id >= 15 && id <=26)
				{
					MotionStatus::m_EnableList[id-1].uID = "DynamicWalking";
				}
				if(id == 1 || id ==2 || id == 7|| id ==8)
					MotionStatus::m_EnableList[id-1].uID = "DynamicWalking";
			}

			MotionManager::GetInstance()->StartTimer();

			printf("Playing.... If you want to stop this demo, Press the ESC Button!\n");
			while(true)
			{
				if(kbhit())
					if(_getch() == 27)
					{
						break;
					}
			}
			printf("Demo terminated by user!\n");
			MotionManager::GetInstance()->StopTimer();
		}
		else if(strcmp(cmd, "exit") == 0)
		{
			return 0;
		}
	}
}


