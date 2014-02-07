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
//#include "framework/motion/modules/testmodule.h"
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
			" forward       : forward walking\n"
			" climbblock    : block climbing \n" \
			"release        : hand release \n"\
			"grab           : hand grab \n"\
			"h              : hand hello\n"\
			"initpose       :  \n"\
			"lookvalve      :   \n"\
			"fastenvalve     :  \n"\
			" exit          : Exit this program \n" \
	);

	//			" c             : walking down the block \n" \
	//			" d             : ??? \n" \//
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
			MotionStatus::m_EnableList[id-1].uID = "PreviewControlWalking";
		}
		if(id == 1 || id ==2 || id == 7|| id ==8)
			MotionStatus::m_EnableList[id-1].uID = "PreviewControlWalking";
	}

	MotionStatus::m_EnableList[27 - 1].uID = "Action";
	MotionStatus::m_EnableList[28 - 1].uID = "Action";
	MotionStatus::m_EnableList[29 - 1].uID = "Action";
	MotionStatus::m_EnableList[30 - 1].uID = "Action";
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

		//<<<<<<< .mine
		else if((id > 10   && id < 15) || id == 27 || id == 28 || id == 29 || id == 30)
			//=======
			//		else if((id > 10   && id < 15) || id == 27 || id == 28 || id == 29 )
			//>>>>>>> .r101
			MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteDWord(id, PRO54::P_GOAL_POSITION_LL,  0, &err);
		else if(id == 30)
			MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteDWord(id, PRO54::P_GOAL_POSITION_LL,  0, &err);
		else if(id == 12)
			MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteDWord(id, PRO54::P_GOAL_POSITION_LL,  0, &err);
		else if(id == 13)
			MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteDWord(id, PRO54::P_GOAL_POSITION_LL,  0, &err);
		else if(id == 14)
			MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteDWord(id, PRO54::P_GOAL_POSITION_LL,  0, &err);


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

		else if(id == 31){
			MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteWord(id, 30, 1955, &err);
			MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteByte(id, 28, 8, &err);
		}

		else if(id == 32){
			MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteWord(id, 30, 2147, &err);
			MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteByte(id, 28, 8, &err);
		}

		else if(id == 33){
			MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteWord(id, 30, 2158, &err);
			MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteByte(id, 28, 8, &err);
		}

		else if(id == 34){
			MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteWord(id, 30, 1971, &err);
			MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteByte(id, 28, 8, &err);
		}
		else if(id == 35){
			MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteWord(id, 30, 1940, &err);
			MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteByte(id, 28, 8, &err);
		}
		else if(id == 36){
			MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteWord(id, 30, 2132, &err);
			MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteByte(id, 28, 8, &err);
		}

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

	for(unsigned int index = 0; index < MotionStatus::m_CurrentJoints.size(); index++)
	{
		int id = MotionStatus::m_CurrentJoints[index].m_ID;
		int err;

		if(id == 31)
			MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteWord(id, 30, 1955, &err);
		else if(id == 32)
			MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteWord(id, 30, 2147, &err);
		else if(id == 33)
			MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteWord(id, 30, 2158, &err);
		else if(id == 34)
			MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteWord(id, 30, 1971, &err);
		else if(id == 35)
			MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteWord(id, 30, 1940, &err);
		else if(id == 36)
			MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteWord(id, 30, 2132, &err);

		usleep(1000);
	}


	PreviewControlWalking::GetInstance()->BALANCE_ENABLE = false;
	PreviewControlWalking::GetInstance()->DEBUG_PRINT = false;

	PreviewControlWalking::GetInstance()->HIP_PITCH_OFFSET = 8.0;//6.0
	PreviewControlWalking::GetInstance()->Initialize();

	PreviewControlWalking::GetInstance()->WALK_STABILIZER_GAIN_RATIO = 2.0;
	PreviewControlWalking::GetInstance()->IMU_GYRO_GAIN_RATIO = 0.0;//7.31*0.01;
	PreviewControlWalking::GetInstance()->FORCE_MOMENT_DISTRIBUTION_RATIO = 0.4;

	PreviewControlWalking::GetInstance()->BALANCE_X_GAIN     = +20.30*0.5*(PreviewControlWalking::GetInstance()->FORCE_MOMENT_DISTRIBUTION_RATIO)*PreviewControlWalking::GetInstance()->WALK_STABILIZER_GAIN_RATIO;
	PreviewControlWalking::GetInstance()->BALANCE_Y_GAIN     = -20.30*(PreviewControlWalking::GetInstance()->FORCE_MOMENT_DISTRIBUTION_RATIO)*PreviewControlWalking::GetInstance()->WALK_STABILIZER_GAIN_RATIO;
	PreviewControlWalking::GetInstance()->BALANCE_PITCH_GAIN = -0.06*0.5*(1-PreviewControlWalking::GetInstance()->FORCE_MOMENT_DISTRIBUTION_RATIO)*PreviewControlWalking::GetInstance()->WALK_STABILIZER_GAIN_RATIO;
	PreviewControlWalking::GetInstance()->BALANCE_ROLL_GAIN  = -0.10*(1-PreviewControlWalking::GetInstance()->FORCE_MOMENT_DISTRIBUTION_RATIO)*PreviewControlWalking::GetInstance()->WALK_STABILIZER_GAIN_RATIO;

	PreviewControlWalking::GetInstance()->FOOT_LANDING_OFFSET_GAIN =   +1.0*0;
	PreviewControlWalking::GetInstance()->FOOT_LANDING_DETECT_N = 50;

	PreviewControlWalking::GetInstance()->SYSTEM_CONTROL_UNIT_TIME_SEC = MotionModule::TIME_UNIT / 1000.0;
	PreviewControlWalking::GetInstance()->FOOT_LANDING_DETECTION_TIME_MAX_SEC = 10.0;

	PreviewControlWalking::GetInstance()->FOOT_ROLL_ADJUSTMENT_ABS_MAX_RAD = 10.0*PI/180;
	PreviewControlWalking::GetInstance()->FOOT_PITCH_ADJUSTMENT_ABS_MAX_RAD = 10.0*PI/180;

	PreviewControlWalking::GetInstance()->COB_X_ADJUSTMENT_ABS_MAX_MM = 50;
	PreviewControlWalking::GetInstance()->COB_Y_ADJUSTMENT_ABS_MAX_MM = 50;


	PreviewControlWalking::GetInstance()->P_GAIN = 64;
	PreviewControlWalking::GetInstance()->I_GAIN = 0;
	PreviewControlWalking::GetInstance()->D_GAIN = 0;

	PreviewControlWalking::GetInstance()->FILTERING_ENABLE = true;
	PreviewControlWalking::GetInstance()->SetSizeforPreviewControl(2.0, 0.5);


	////Action Init and File Load
	//Action::GetInstance()->LoadFile("motion_4096.bin");


	////AddModule
	MotionManager::GetInstance()->AddModule((MotionModule*)PreviewControlWalking::GetInstance());
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
	PreviewControlWalking::GetInstance()->SetInitAngleinRad(RollInitAngleRad/((double)InitAngleWindowSize), PitchInitAngleRad/((double)InitAngleWindowSize));

	printf("Setting Init Angle is completed %f %f!\n", RollInitAngleRad/((double)InitAngleWindowSize),  PitchInitAngleRad/((double)InitAngleWindowSize));

	SetTheMotionEnableList();

	StepData stp0, stp1, stp2, stp3, stp4, stp5;
	//we need to make a Initialize func
	stp0.PositionData.bMovingFoot = NFootMove;        stp0.PositionData.dFootHeight = 0.0;               stp0.PositionData.dZ_Swap_Amplitude = 0.0;
	stp0.PositionData.dShoulderSwingGain = 0.05;      stp0.PositionData.dElbowSwingGain = 0.1;
	stp0.PositionData.stRightFootPosition.x = 0.0;    stp0.PositionData.stRightFootPosition.y = -95.0;   stp0.PositionData.stRightFootPosition.z = 0.0;
	stp0.PositionData.stRightFootPosition.roll = 0.0; stp0.PositionData.stRightFootPosition.pitch = 0.0; stp0.PositionData.stRightFootPosition.yaw = 0.0;
	stp0.PositionData.stBodyPosition.z = 670.0;
	stp0.PositionData.stBodyPosition.roll = 0.0;      stp0.PositionData.stBodyPosition.pitch = 0.0;      stp0.PositionData.stBodyPosition.yaw = 0.0;
	stp0.PositionData.stLeftFootPosition.x = 0.0;     stp0.PositionData.stLeftFootPosition.y = 95.0;     stp0.PositionData.stLeftFootPosition.z = 0.0;
	stp0.PositionData.stLeftFootPosition.roll = 0.0;  stp0.PositionData.stLeftFootPosition.pitch = 0.0;  stp0.PositionData.stLeftFootPosition.yaw = 0.0;

	stp0.TimeData.bWalkingState = InWalkingStarting;
	stp0.TimeData.dAbsStepTime = 3000.0;    stp0.TimeData.dDSPratio = 0.2;
	stp0.TimeData.sigmoid_ratio_x = 1.0;    stp0.TimeData.sigmoid_ratio_y = 1.0;     stp0.TimeData.sigmoid_ratio_z = 1.0;
	stp0.TimeData.sigmoid_ratio_roll = 1.0; stp0.TimeData.sigmoid_ratio_pitch = 1.0; stp0.TimeData.sigmoid_ratio_yaw = 1.0;
	stp0.TimeData.sigmoid_distortion_x = 1.0;    stp0.TimeData.sigmoid_distortion_y = 1.0;     stp0.TimeData.sigmoid_distortion_z = 1.0;
	stp0.TimeData.sigmoid_distortion_roll = 1.0; stp0.TimeData.sigmoid_distortion_pitch = 1.0; stp0.TimeData.sigmoid_distortion_yaw = 1.0;



	stp1.PositionData.bMovingFoot = RFootMove;        stp1.PositionData.dFootHeight = 80.0;               stp1.PositionData.dZ_Swap_Amplitude = 10.0;
	stp1.PositionData.dShoulderSwingGain = 0.05;      stp1.PositionData.dElbowSwingGain = 0.1;
	stp1.PositionData.stRightFootPosition.x = 140.0;  stp1.PositionData.stRightFootPosition.y = -95.0;   stp1.PositionData.stRightFootPosition.z = 0.0;
	stp1.PositionData.stRightFootPosition.roll = 0.0; stp1.PositionData.stRightFootPosition.pitch = 0.0; stp1.PositionData.stRightFootPosition.yaw = 0.0;
	stp1.PositionData.stBodyPosition.z = 670.0;
	stp1.PositionData.stBodyPosition.roll = 0.0;      stp1.PositionData.stBodyPosition.pitch = 0.0;      stp1.PositionData.stBodyPosition.yaw = 0.0;
	stp1.PositionData.stLeftFootPosition.x = 0.0;     stp1.PositionData.stLeftFootPosition.y = 95.0;     stp1.PositionData.stLeftFootPosition.z = 0.0;
	stp1.PositionData.stLeftFootPosition.roll = 0.0;  stp1.PositionData.stLeftFootPosition.pitch = 0.0;  stp1.PositionData.stLeftFootPosition.yaw = 0.0;

	stp1.TimeData.bWalkingState = InWalking;
	stp1.TimeData.dAbsStepTime = 3850.0;    stp1.TimeData.dDSPratio = 0.2;
	stp1.TimeData.sigmoid_ratio_x = 1.0;    stp1.TimeData.sigmoid_ratio_y = 1.0;     stp1.TimeData.sigmoid_ratio_z = 1.0;
	stp1.TimeData.sigmoid_ratio_roll = 1.0; stp1.TimeData.sigmoid_ratio_pitch = 1.0; stp1.TimeData.sigmoid_ratio_yaw = 1.0;
	stp1.TimeData.sigmoid_distortion_x = 1.0;    stp1.TimeData.sigmoid_distortion_y = 1.0;     stp1.TimeData.sigmoid_distortion_z = 1.0;
	stp1.TimeData.sigmoid_distortion_roll = 1.0; stp1.TimeData.sigmoid_distortion_pitch = 1.0; stp1.TimeData.sigmoid_distortion_yaw = 1.0;



	stp2.PositionData.bMovingFoot = LFootMove;        stp2.PositionData.dFootHeight = 80.0;               stp2.PositionData.dZ_Swap_Amplitude = 10.0;
	stp2.PositionData.dShoulderSwingGain = 0.05;      stp2.PositionData.dElbowSwingGain = 0.1;
	stp2.PositionData.stRightFootPosition.x = 140.0;  stp2.PositionData.stRightFootPosition.y = -95.0;   stp2.PositionData.stRightFootPosition.z = 0.0;
	stp2.PositionData.stRightFootPosition.roll = 0.0; stp2.PositionData.stRightFootPosition.pitch = 0.0; stp2.PositionData.stRightFootPosition.yaw = 0.0;
	stp2.PositionData.stBodyPosition.z = 670.0;
	stp2.PositionData.stBodyPosition.roll = 0.0;      stp2.PositionData.stBodyPosition.pitch = 0.0;      stp2.PositionData.stBodyPosition.yaw = 0.0;
	stp2.PositionData.stLeftFootPosition.x = 280.0;     stp2.PositionData.stLeftFootPosition.y = 95.0;   stp2.PositionData.stLeftFootPosition.z = 0.0;
	stp2.PositionData.stLeftFootPosition.roll = 0.0;  stp2.PositionData.stLeftFootPosition.pitch = 0.0;  stp2.PositionData.stLeftFootPosition.yaw = 0.0;

	stp2.TimeData.bWalkingState = InWalking;
	stp2.TimeData.dAbsStepTime = 4700.0;    stp2.TimeData.dDSPratio = 0.2;
	stp2.TimeData.sigmoid_ratio_x = 1.0;    stp2.TimeData.sigmoid_ratio_y = 1.0;     stp2.TimeData.sigmoid_ratio_z = 1.0;
	stp2.TimeData.sigmoid_ratio_roll = 1.0; stp2.TimeData.sigmoid_ratio_pitch = 1.0; stp2.TimeData.sigmoid_ratio_yaw = 1.0;
	stp2.TimeData.sigmoid_distortion_x = 1.0;    stp2.TimeData.sigmoid_distortion_y = 1.0;     stp2.TimeData.sigmoid_distortion_z = 1.0;
	stp2.TimeData.sigmoid_distortion_roll = 1.0; stp2.TimeData.sigmoid_distortion_pitch = 1.0; stp2.TimeData.sigmoid_distortion_yaw = 1.0;



	stp3.PositionData.bMovingFoot = RFootMove;        stp3.PositionData.dFootHeight = 0.0;               stp3.PositionData.dZ_Swap_Amplitude = 10.0;
	stp3.PositionData.dShoulderSwingGain = 0.05;      stp3.PositionData.dElbowSwingGain = 0.1;
	stp3.PositionData.stRightFootPosition.x = 280.0;  stp3.PositionData.stRightFootPosition.y = -95.0;   stp3.PositionData.stRightFootPosition.z = 0.0;
	stp3.PositionData.stRightFootPosition.roll = 0.0; stp3.PositionData.stRightFootPosition.pitch = 0.0; stp3.PositionData.stRightFootPosition.yaw = 0.0;
	stp3.PositionData.stBodyPosition.z = 670.0;
	stp3.PositionData.stBodyPosition.roll = 0.0;      stp3.PositionData.stBodyPosition.pitch = 0.0;      stp3.PositionData.stBodyPosition.yaw = 0.0;
	stp3.PositionData.stLeftFootPosition.x = 280.0;     stp3.PositionData.stLeftFootPosition.y = 95.0;   stp3.PositionData.stLeftFootPosition.z = 0.0;
	stp3.PositionData.stLeftFootPosition.roll = 0.0;  stp3.PositionData.stLeftFootPosition.pitch = 0.0;  stp3.PositionData.stLeftFootPosition.yaw = 0.0;

	stp3.TimeData.bWalkingState = InWalking;
	stp3.TimeData.dAbsStepTime = 5550.0;    stp3.TimeData.dDSPratio = 0.2;
	stp3.TimeData.sigmoid_ratio_x = 1.0;    stp3.TimeData.sigmoid_ratio_y = 1.0;     stp3.TimeData.sigmoid_ratio_z = 1.0;
	stp3.TimeData.sigmoid_ratio_roll = 1.0; stp3.TimeData.sigmoid_ratio_pitch = 1.0; stp3.TimeData.sigmoid_ratio_yaw = 1.0;
	stp3.TimeData.sigmoid_distortion_x = 1.0;    stp3.TimeData.sigmoid_distortion_y = 1.0;     stp3.TimeData.sigmoid_distortion_z = 1.0;
	stp3.TimeData.sigmoid_distortion_roll = 1.0; stp3.TimeData.sigmoid_distortion_pitch = 1.0; stp3.TimeData.sigmoid_distortion_yaw = 1.0;



	stp4.PositionData.bMovingFoot = NFootMove;        stp4.PositionData.dFootHeight = 0.0;               stp4.PositionData.dZ_Swap_Amplitude = 0.0;
	stp4.PositionData.dShoulderSwingGain = 0.05;      stp4.PositionData.dElbowSwingGain = 0.1;
	stp4.PositionData.stRightFootPosition.x = 280.0;  stp4.PositionData.stRightFootPosition.y = -95.0;   stp4.PositionData.stRightFootPosition.z = 0.0;
	stp4.PositionData.stRightFootPosition.roll = 0.0; stp4.PositionData.stRightFootPosition.pitch = 0.0; stp4.PositionData.stRightFootPosition.yaw = 0.0;
	stp4.PositionData.stBodyPosition.z = 670.0;
	stp4.PositionData.stBodyPosition.roll = 0.0;      stp4.PositionData.stBodyPosition.pitch = 0.0;      stp4.PositionData.stBodyPosition.yaw = 0.0;
	stp4.PositionData.stLeftFootPosition.x = 280.0;   stp4.PositionData.stLeftFootPosition.y = 95.0;   stp4.PositionData.stLeftFootPosition.z = 0.0;
	stp4.PositionData.stLeftFootPosition.roll = 0.0;  stp4.PositionData.stLeftFootPosition.pitch = 0.0;  stp4.PositionData.stLeftFootPosition.yaw = 0.0;

	stp4.TimeData.bWalkingState = InWalkingEnding;
	stp4.TimeData.dAbsStepTime = 8550.0;    stp4.TimeData.dDSPratio = 0.2;
	stp4.TimeData.sigmoid_ratio_x = 1.0;    stp4.TimeData.sigmoid_ratio_y = 1.0;     stp4.TimeData.sigmoid_ratio_z = 1.0;
	stp4.TimeData.sigmoid_ratio_roll = 1.0; stp4.TimeData.sigmoid_ratio_pitch = 1.0; stp4.TimeData.sigmoid_ratio_yaw = 1.0;
	stp4.TimeData.sigmoid_distortion_x = 1.0;    stp4.TimeData.sigmoid_distortion_y = 1.0;     stp4.TimeData.sigmoid_distortion_z = 1.0;
	stp4.TimeData.sigmoid_distortion_roll = 1.0; stp4.TimeData.sigmoid_distortion_pitch = 1.0; stp4.TimeData.sigmoid_distortion_yaw = 1.0;

	PreviewControlWalking::GetInstance()->AddStepData(stp0);
	PreviewControlWalking::GetInstance()->AddStepData(stp1);
	PreviewControlWalking::GetInstance()->AddStepData(stp2);
	PreviewControlWalking::GetInstance()->AddStepData(stp3);
	PreviewControlWalking::GetInstance()->AddStepData(stp4);
	//PreviewControlWalking::GetInstance()->Initialize();
	printf("press any key to start");
	_getch();
	PreviewControlWalking::GetInstance()->Start();
	MotionManager::GetInstance()->StartTimer();
	while(true)
	{
		if(kbhit())
			if(_getch() == 27)
				break;
		usleep(8000);
	}
}


