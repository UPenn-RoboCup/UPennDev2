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
#include <ncurses.h>
#include <libgen.h>
//#include "streamer/mjpg_streamer.h"
#include "framework/motion/modules/testmodule.h"
#include "framework/Thor.h"

#define INI_FILE_PATH "config.ini"

#define PI (3.14159265)

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

void Help()
{
	fprintf(stderr, "\n");
	fprintf(stderr, "COMMAND: \n\n" \
			" a             : forward walking\n"
			" b             : block climbing \n" \
			" c             : walking down the block \n" \
			" d             : ??? \n" \
			"release        : hand release \n"\
			"grab           : hand grab \n"\
			" exit          : Exit this program \n" \
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


void SetTheMotionEnableList(void)
{
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
}

int main(int argc, char *argv[])
{
	fprintf(stderr, "\n***********************************************************************\n");
	fprintf(stderr,   "*                         Demo for RoboWorld                          *\n");
	fprintf(stderr,   "***********************************************************************\n\n");


	////////////////////////////////////////////// Motion Manager Initializing //////////////////////////////////////////////
	if(MotionManager::GetInstance()->Initialize() == false)
	{
		printf("Fail to initialize Motion Manager!\n");
		return 0;
	}

	minIni* ini = new minIni("config.ini");
	MotionManager::GetInstance()->LoadINISettings(ini);


	////////////////////////////////////////////////////// Speed Down //////////////////////////////////////////////////////
	for(unsigned int index = 0; index < MotionStatus::m_CurrentJoints.size(); index++)
	{
		int id = MotionStatus::m_CurrentJoints[index].m_ID;

		int err;
		MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteByte(id, PRO54::P_TORQUE_ENABLE, 1, &err);
		MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteDWord(id, PRO54::P_GOAL_ACCELATION_LL, 4, &err);
		MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteDWord(id, PRO54::P_GOAL_VELOCITY_LL, 2000, &err);
		printf("id : %d  ", id);
		PrintErrorCode(err);
		usleep(1000);
	}

	//////////////////////////////////// Ins Initializing and Start Update of Initialize ////////////////////////////////////
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



	printf("Press the any button to move first pose!\n");
	_getch();

	for(unsigned int index = 0; index < MotionStatus::m_CurrentJoints.size(); index++)
	{
		int id = MotionStatus::m_CurrentJoints[index].m_ID;

		int err = 0;
		if( id == 1)
			MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteDWord(id, PRO54::P_GOAL_POSITION_LL, -62750, &err);
		else if(id == 2)
			MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteDWord(id, PRO54::P_GOAL_POSITION_LL, 62750, &err);
		else if(id == 3)
			MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteDWord(id, PRO54::P_GOAL_POSITION_LL, -109520, &err);
		else if(id == 4)
			MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteDWord(id, PRO54::P_GOAL_POSITION_LL, 109520, &err);
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

		else if((id > 10   && id < 15) || id == 27)
			MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteDWord(id, PRO54::P_GOAL_POSITION_LL,  0, &err);
		else if(id == 28)
			MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteDWord(id, PRO54::P_GOAL_POSITION_LL, 10000, &err);
		else if(id == 12)
			MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteDWord(id, PRO54::P_GOAL_POSITION_LL,  0, &err);
		else if(id == 13)
			MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteDWord(id, PRO54::P_GOAL_POSITION_LL,  0, &err);
		else if(id == 14)
			MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteDWord(id, PRO54::P_GOAL_POSITION_LL,  0, &err);

//		else if(id == 15)
//			MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteDWord(id, PRO54::P_GOAL_POSITION_LL,  0 + MotionManager::GetInstance()->m_Offset[14], &err);
//		else if(id == 17)
//			MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteDWord(id, PRO54::P_GOAL_POSITION_LL,  2629 + MotionManager::GetInstance()->m_Offset[16], &err);
//		else if(id == 19)
//			MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteDWord(id, PRO54::P_GOAL_POSITION_LL,  49364+ MotionManager::GetInstance()->m_Offset[18], &err);
//		else if(id == 21)
//			MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteDWord(id, PRO54::P_GOAL_POSITION_LL,  -17477 - 1700 + MotionManager::GetInstance()->m_Offset[20], &err);
//		else if(id == 23)
//			MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteDWord(id, PRO54::P_GOAL_POSITION_LL,  -41922 + 1029 + MotionManager::GetInstance()->m_Offset[22], &err);
//		else if(id == 25)
//			MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteDWord(id, PRO54::P_GOAL_POSITION_LL,  2296 + 345 + MotionManager::GetInstance()->m_Offset[24], &err);
//
//		else if(id == 16)
//			MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteDWord(id, PRO54::P_GOAL_POSITION_LL,  0  + MotionManager::GetInstance()->m_Offset[15], &err);
//		else if(id == 18)
//			MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteDWord(id, PRO54::P_GOAL_POSITION_LL,  -4729 + 2150 + MotionManager::GetInstance()->m_Offset[17], &err);
//		else if(id == 20)
//			MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteDWord(id, PRO54::P_GOAL_POSITION_LL,  -49566+200 + MotionManager::GetInstance()->m_Offset[19], &err);
//		else if(id == 22)
//			MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteDWord(id, PRO54::P_GOAL_POSITION_LL,  17980+1200 + MotionManager::GetInstance()->m_Offset[21], &err);
//		else if(id == 24)
//			MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteDWord(id, PRO54::P_GOAL_POSITION_LL,  42110-1215 + MotionManager::GetInstance()->m_Offset[23], &err);
//		else if(id == 26)
//			MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteDWord(id, PRO54::P_GOAL_POSITION_LL,  -3061+496 + MotionManager::GetInstance()->m_Offset[25], &err);

		else if(id == 15)
			MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteDWord(id, PRO54::P_GOAL_POSITION_LL,  0+ MotionManager::GetInstance()->m_Offset[14], &err);
		else if(id == 17)
			MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteDWord(id, PRO54::P_GOAL_POSITION_LL,  0.050872938*251000.0/PI  + MotionManager::GetInstance()->m_Offset[16], &err);
		else if(id == 19)
			MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteDWord(id, PRO54::P_GOAL_POSITION_LL,  0.511256734*251000.0/PI + 8.0*251000.0/180.0  + MotionManager::GetInstance()->m_Offset[18], &err);
		else if(id == 21)
			MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteDWord(id, PRO54::P_GOAL_POSITION_LL,  -0.236447918*251000.0/PI+ MotionManager::GetInstance()->m_Offset[20], &err);
		else if(id == 23)
			MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteDWord(id, PRO54::P_GOAL_POSITION_LL,  -0.510589347*251000.0/PI+ MotionManager::GetInstance()->m_Offset[22], &err);
		else if(id == 25)
			MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteDWord(id, PRO54::P_GOAL_POSITION_LL,  0.050872949*251000.0/PI+ MotionManager::GetInstance()->m_Offset[24], &err);

		else if(id == 16)
			MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteDWord(id, PRO54::P_GOAL_POSITION_LL,  0+ MotionManager::GetInstance()->m_Offset[15], &err);
		else if(id == 18)
			MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteDWord(id, PRO54::P_GOAL_POSITION_LL,  -0.050489437*251000.0/PI+ MotionManager::GetInstance()->m_Offset[17], &err);
		else if(id == 20)
			MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteDWord(id, PRO54::P_GOAL_POSITION_LL,  -0.511301342*251000.0/PI - 8.0*251000.0/180.0+ MotionManager::GetInstance()->m_Offset[19], &err);
		else if(id == 22)
			MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteDWord(id, PRO54::P_GOAL_POSITION_LL,  0.236537121*251000.0/PI+ MotionManager::GetInstance()->m_Offset[21], &err);
		else if(id == 24)
			MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteDWord(id, PRO54::P_GOAL_POSITION_LL,  0.510633942*251000.0/PI+ MotionManager::GetInstance()->m_Offset[23], &err);
		else if(id == 26)
			MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteDWord(id, PRO54::P_GOAL_POSITION_LL,  -0.050489448*251000.0/PI+ MotionManager::GetInstance()->m_Offset[25], &err);

		usleep(1000);
	}


//	usleep(8000);
//	for(unsigned int index = 0; index < MotionStatus::m_CurrentJoints.size(); index++)
//	{
//		usleep(1);
//		int id = MotionStatus::m_CurrentJoints[index].m_ID;
//
//		if(id > 14 && id <27)
//			MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteWord(id, PRO54::P_VELOCITY_I_GAIN_L, 32, 0);
//	}


	usleep(7000000);
	printf("Press the any button. After the moving stop!\n");
	_getch();



	MotionManager::GetInstance()->Reinitialize();
	DynamicWalking::GetInstance()->BALANCE_ENABLE = true;
	DynamicWalking::GetInstance()->DEBUG_PRINT = false;
//	DynamicWalking::GetInstance()->BALANCE_KNEE_GAIN = 0.3*0.5*2.0;
//	DynamicWalking::GetInstance()->BALANCE_ANKLE_PITCH_GAIN = 0.9*0.5*2.0;
//	DynamicWalking::GetInstance()->BALANCE_HIP_ROLL_GAIN = 0.5*2.0;
//	DynamicWalking::GetInstance()->BALANCE_ANKLE_ROLL_GAIN = 1.0*2.0;

	DynamicWalking::GetInstance()->HIP_PITCH_OFFSET = 8.0;//6.0
	DynamicWalking::GetInstance()->FZ_WINDOW_SIZE = 3;

	DynamicWalking::GetInstance()->SetFileName("ForwardWalkingWP.txt", "ForwardWalkingEP.txt", "ForwardWalkingBalancingIdxData.txt");
	DynamicWalking::GetInstance()->Initialize();

	DynamicWalking::GetInstance()->WALK_STABILIZER_GAIN_RATIO = 2.0;
	DynamicWalking::GetInstance()->IMU_GYRO_GAIN_RATIO = 0.0;//7.31*0.01;
	DynamicWalking::GetInstance()->FORCE_MOMENT_DISTRIBUTION_RATIO = 0.4;

	DynamicWalking::GetInstance()->BALANCE_X_GAIN     = +20.30*0.5*(DynamicWalking::GetInstance()->FORCE_MOMENT_DISTRIBUTION_RATIO)*DynamicWalking::GetInstance()->WALK_STABILIZER_GAIN_RATIO;
	DynamicWalking::GetInstance()->BALANCE_Y_GAIN     = -20.30*(DynamicWalking::GetInstance()->FORCE_MOMENT_DISTRIBUTION_RATIO)*DynamicWalking::GetInstance()->WALK_STABILIZER_GAIN_RATIO;
	DynamicWalking::GetInstance()->BALANCE_PITCH_GAIN = -0.06*0.5*(1-DynamicWalking::GetInstance()->FORCE_MOMENT_DISTRIBUTION_RATIO)*DynamicWalking::GetInstance()->WALK_STABILIZER_GAIN_RATIO;
	DynamicWalking::GetInstance()->BALANCE_ROLL_GAIN  = -0.10*1.0*(1-DynamicWalking::GetInstance()->FORCE_MOMENT_DISTRIBUTION_RATIO)*DynamicWalking::GetInstance()->WALK_STABILIZER_GAIN_RATIO;

	DynamicWalking::GetInstance()->FOOT_LANDING_OFFSET_GAIN =   +1.0*0;
	DynamicWalking::GetInstance()->FOOT_LANDING_DETECT_N = 50;

	DynamicWalking::GetInstance()->SYSTEM_CONTROL_UNIT_TIME_SEC = MotionModule::TIME_UNIT / 1000.0;
	DynamicWalking::GetInstance()->FOOT_LANDING_DETECTION_TIME_MAX_SEC = 10.0;

	DynamicWalking::GetInstance()->FOOT_ROLL_ADJUSTMENT_ABS_MAX_RAD = 10.0*PI/180;
	DynamicWalking::GetInstance()->FOOT_PITCH_ADJUSTMENT_ABS_MAX_RAD = 10.0*PI/180;

	DynamicWalking::GetInstance()->COB_X_ADJUSTMENT_ABS_MAX_MM = 50;
	DynamicWalking::GetInstance()->COB_Y_ADJUSTMENT_ABS_MAX_MM = 50;



	DynamicWalking::GetInstance()->P_GAIN = 64;
	DynamicWalking::GetInstance()->I_GAIN = 0;
	DynamicWalking::GetInstance()->D_GAIN = 0;


	MotionManager::GetInstance()->AddModule((MotionModule*)DynamicWalking::GetInstance());
	MotionManager::GetInstance()->AddModule((MotionModule*)Test::GetInstance());
	//MotionManager::GetInstance()->AddModule((MotionModule*)Action::GetInstance());

	for(unsigned int index = 0; index < MotionStatus::m_CurrentJoints.size(); index++)
	{
		int id = MotionStatus::m_CurrentJoints[index].m_ID;

		int err;
		MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteDWord(id, PRO54::P_GOAL_ACCELATION_LL, 0, &err);
		MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteDWord(id, PRO54::P_GOAL_VELOCITY_LL, 0, &err);
		printf("id : %d  ", id);
		PrintErrorCode(err);
		usleep(1000);
	}


	printf("Press the any key. After the fall down on the ground!\n");
	_getch();

	double RollInitAngleRad = 0.0, PitchInitAngleRad = 0.0;
	int InitAngleWindowSize = 50;
	for(int index =0; index < InitAngleWindowSize; index++)
	{
		RollInitAngleRad += MotionStatus::EulerAngleX;
		PitchInitAngleRad += MotionStatus::EulerAngleY;
		usleep(4000);
	}
	DynamicWalking::GetInstance()->SetInitAngleinRad(RollInitAngleRad/((double)InitAngleWindowSize), PitchInitAngleRad/((double)InitAngleWindowSize));

	printf("Setting Init Angle is completed %f %f!\n", RollInitAngleRad/((double)InitAngleWindowSize),  PitchInitAngleRad/((double)InitAngleWindowSize));

	DynamicWalking::GetInstance()->idxIncreasement = 1.0;
	//MotionManager::GetInstance()->StartTimer();

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

			DynamicWalking::GetInstance()->SetFileName("ForwardWalkingWP.txt", "ForwardWalkingEP.txt", "ForwardWalkingBalancingIdxData.txt");
			DynamicWalking::GetInstance()->ReInitialize();

			SetTheMotionEnableList();
			DynamicWalking::GetInstance()->Start();
			if(MotionManager::GetInstance()->IsTimerRunning() == false)
				MotionManager::GetInstance()->StartTimer();

			printf("Playing.... If you want to stop this demo, Press the ESC Button!\n");
			while(true)
			{
				if(kbhit())
					if(_getch() == 27)
					{
						DynamicWalking::GetInstance()->Stop();
						break;
					}
			}
			printf("Demo terminated by user!\n");
			//MotionManager::GetInstance()->StopTimer();
		}
		else if(strcmp(cmd, "b") == 0)
		{
			printf("Start Climbing Block Demo!\n");
			_getch();

			DynamicWalking::GetInstance()->SetFileName("ClimbBlockWP.txt", "ClimbBlockEP.txt", "ClimbBlockBalancingIdxData.txt");
			DynamicWalking::GetInstance()->ReInitialize();

			//DynamicWalking::GetInstance()->HIP_PITCH_OFFSET = 4.0;//6.0
			SetTheMotionEnableList();
			DynamicWalking::GetInstance()->Start();
			if(MotionManager::GetInstance()->IsTimerRunning() == false)
				MotionManager::GetInstance()->StartTimer();

			printf("Playing.... If you want to stop this demo, Press the ESC Button!\n");
			while(true)
			{
				if(kbhit())
					if(_getch() == 27)
					{
						DynamicWalking::GetInstance()->Stop();
						break;
					}
			}
			printf("Demo terminated by user!\n");
			//MotionManager::GetInstance()->StopTimer();

		}
		else if(strcmp(cmd, "c")==0)
		{
//			printf("Start Walking Down Block Demo!\n");
//			_getch();
//
//			DynamicWalking::GetInstance()->SetFileName("DownBlock.txt");
//			DynamicWalking::GetInstance()->ReInitialize();
//
//			SetTheMotionEnableList();
//			DynamicWalking::GetInstance()->Start();
//			//MotionManager::GetInstance()->StartTimer();
//
//			printf("Playing.... If you want to stop this demo, Press the ESC Button!\n");
//			while(true)
//			{
//				if(kbhit())
//					if(_getch() == 27)
//					{
//						DynamicWalking::GetInstance()->Stop();
//						break;
//					}
//			}
//			printf("Demo terminated by user!\n");
//			//MotionManager::GetInstance()->StopTimer();
		}
		else if(strcmp(cmd, "d") == 0)
		{
//			printf("Start ????? Demo!\n");
//			_getch();
//
//			DynamicWalking::GetInstance()->SetFileName("Angles.txt");
//			DynamicWalking::GetInstance()->ReInitialize();
//
//			SetTheMotionEnableList();
//			DynamicWalking::GetInstance()-> Start();
//			//MotionManager::GetInstance()->StartTimer();
//
//			printf("Playing.... If you want to stop this demo, Press the ESC Button!\n");
//			while(true)
//			{
//				if(kbhit())
//					if(_getch() == 27)
//					{
//						DynamicWalking::GetInstance()->Stop();
//						break;
//					}
//			}
//			printf("Demo terminated by user!\n");
//			//MotionManager::GetInstance()->StopTimer();
		}
		else if(strcmp(cmd, "e") == 0)
		{
			printf("Start In place Walking Demo!\n");
			_getch();

			DynamicWalking::GetInstance()->SetFileName("placeWalkingWP.txt", "placeWalkingEP.txt", "placeWalkingBalancingIdxData.txt");
			DynamicWalking::GetInstance()->ReInitialize();

			SetTheMotionEnableList();
			DynamicWalking::GetInstance()->Start();
			if(MotionManager::GetInstance()->IsTimerRunning() == false)
				MotionManager::GetInstance()->StartTimer();

			printf("Playing.... If you want to stop this demo, Press the ESC Button!\n");
			while(true)
			{
				if(kbhit())
					if(_getch() == 27)
					{
						DynamicWalking::GetInstance()->Stop();
						break;
					}
			}
			printf("Demo terminated by user!\n");
		}
		else if(strcmp(cmd, "p") == 0)
		{
			if(DynamicWalking::GetInstance()->idxIncreasement > 0.9)
				DynamicWalking::GetInstance()->idxIncreasement = 0.2;
			else
				DynamicWalking::GetInstance()->idxIncreasement = 1.0;
		}
		else if(strcmp(cmd, "release") == 0){
//			MotionStatus::m_CurrentJoints[30].m_Value = 2048;
//			MotionStatus::m_CurrentJoints[31].m_Value = 2048;
//			MotionStatus::m_CurrentJoints[32].m_Value = 2048;
//			MotionStatus::m_CurrentJoints[33].m_Value = 2048;
//			MotionStatus::m_CurrentJoints[34].m_Value = 2048;
//			MotionStatus::m_CurrentJoints[35].m_Value = 2048;

			Test::GetInstance()->m_RobotInfo[30].m_Value = 2048;
			Test::GetInstance()->m_RobotInfo[31].m_Value = 2048;
			Test::GetInstance()->m_RobotInfo[32].m_Value = 2048;
			Test::GetInstance()->m_RobotInfo[33].m_Value = 2048;
			Test::GetInstance()->m_RobotInfo[34].m_Value = 2048;
			Test::GetInstance()->m_RobotInfo[35].m_Value = 2048;
			printf("\n release!!\n");
		}
		else if(strcmp(cmd, "grab") == 0) {
//			MotionStatus::m_CurrentJoints[30].m_Value = 1955;
//			MotionStatus::m_CurrentJoints[31].m_Value = 2147;
//
//			MotionStatus::m_CurrentJoints[32].m_Value = 2158;
//			MotionStatus::m_CurrentJoints[33].m_Value = 1971;
//
//			MotionStatus::m_CurrentJoints[34].m_Value = 1940;
//			MotionStatus::m_CurrentJoints[35].m_Value = 2132;

			Test::GetInstance()->m_RobotInfo[30].m_Value = 1955;
			Test::GetInstance()->m_RobotInfo[31].m_Value = 2147;

			Test::GetInstance()->m_RobotInfo[32].m_Value = 2158;
			Test::GetInstance()->m_RobotInfo[33].m_Value = 1971;

			Test::GetInstance()->m_RobotInfo[34].m_Value = 1940;
			Test::GetInstance()->m_RobotInfo[35].m_Value = 2132;

			printf("\n grab!!\n");
		}
		else if(strcmp(cmd, "h") == 0) {
//			MotionStatus::m_CurrentJoints[30].m_Value = 2265;
//			MotionStatus::m_CurrentJoints[32].m_Value = 1730;
//			MotionStatus::m_CurrentJoints[34].m_Value = 2369;
//
//			MotionStatus::m_CurrentJoints[31].m_Value = 2048;
//			MotionStatus::m_CurrentJoints[33].m_Value = 2048;
//			MotionStatus::m_CurrentJoints[35].m_Value = 2048;

			Test::GetInstance()->m_RobotInfo[30].m_Value = 2265;
			Test::GetInstance()->m_RobotInfo[32].m_Value = 1730;
			Test::GetInstance()->m_RobotInfo[34].m_Value = 2369;

			Test::GetInstance()->m_RobotInfo[31].m_Value = 2048;
			Test::GetInstance()->m_RobotInfo[33].m_Value = 2048;
			Test::GetInstance()->m_RobotInfo[35].m_Value = 2048;

			printf("\n hello!!\n");
		}
		else if(strcmp(cmd, "exit") == 0) {
			return 0;
		}
	}
}


