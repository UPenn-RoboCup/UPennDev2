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
#define ANG2RAD PI/180.0

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
			MotionStatus::m_EnableList[id-1].uID = "RecursiveWalking";
		}
		if(id == 1 || id ==2 || id == 7|| id ==8)
			MotionStatus::m_EnableList[id-1].uID = "RecursiveWalking";
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


	int dir_output[16];
	double InitAngle[16];

	//for thor
	dir_output[0] = -1; dir_output[1] = -1; dir_output[2] = -1; dir_output[3] = -1; dir_output[4] =  1; dir_output[5] = 1;
	dir_output[6] = -1; dir_output[7] = -1; dir_output[8] =  1; dir_output[9] =  1; dir_output[10]= -1; dir_output[11] = 1;
	dir_output[12] = -1; dir_output[13] = 1;  dir_output[14] = -1; dir_output[15] =  1;
	InitAngle[0]  =   0.0;  InitAngle[1]  =  0.0;  InitAngle[2]  =  5.7106;  InitAngle[3] =  33.5788; InitAngle[4]  = -5.7106; InitAngle[5]  = 0.0;
	InitAngle[6]  =   0.0;  InitAngle[7]  =  0.0;  InitAngle[8]  = -5.7106;  InitAngle[9] = -33.5788; InitAngle[10] =  5.7106; InitAngle[11] = 0.0;
	InitAngle[12] = -45.0,  InitAngle[13] = 45.0;  InitAngle[14] =  45.0;    InitAngle[15] =  -45.0;

	double angle[16];
	int outValue[16];

	matd GtoCOB = GetTransformMatrix(0, 0, 670, 0, 0, 0 );
	matd GtoRF = GetTransformMatrix(0, -95, 0, 0, 0, 0);
	matd GtoLF = GetTransformMatrix(0,  95, 0, 0, 0, 0);

	matd RHtoCOB = GetTranslationMatrix(0,  Kinematics::LEG_SIDE_OFFSET*0.5, 0);
	matd LHtoCOB = GetTranslationMatrix(0, -Kinematics::LEG_SIDE_OFFSET*0.5, 0);

	matd COBtoG = GetTransformMatrixInverse(GtoCOB);
	matd RHtoRF = RHtoCOB*COBtoG*GtoRF;
	matd LHtoLF = LHtoCOB*COBtoG*GtoLF;

	Pose3D epr, epl;

	epr = GetPose3DfromTransformMatrix(RHtoRF);
	epl = GetPose3DfromTransformMatrix(LHtoLF);

	if(RecursiveWalking::GetInstance()->computeIK(&angle[0], epr.x, epr.y, epr.z+Kinematics::LEG_LENGTH, epr.roll, epr.pitch, epr.yaw) == false) {
		printf("IKsolve failed\n");
		return 0;
	}

	if(RecursiveWalking::GetInstance()->computeIK(&angle[6], epl.x, epl.y, epl.z+Kinematics::LEG_LENGTH, epl.roll, epl.pitch, epl.yaw) == false) {
		printf("IKsolve failed\n");
		return 0;
	}

	for(int idx = 0; idx < 6; idx++)
	{
		angle[idx] = (double)dir_output[idx]*angle[idx]*180.0/PI + InitAngle[idx];
		angle[idx+6] = (double)dir_output[idx+6]*angle[idx+6]*180.0/PI + InitAngle[idx+6];
	}


	for(int idx = 0; idx < 16; idx++)
	{
		//		double offset = (double)dir[i] * angle[i] * (g_pro54->MAX_VALUE)/180.0;
		//        outValue[i] = g_pro54->Angle2Value(initAngle[i]) + (int)offset;
		outValue[idx] = 251000.0*(angle[idx])/180.0;
	}

	outValue[2] -= (double)dir_output[2] * 8.0 * 251000.0/180.0;
	outValue[8] -= (double)dir_output[8] * 8.0 * 251000.0/180.0;

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
			MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteDWord(id, PRO54::P_GOAL_POSITION_LL,  outValue[0] + MotionManager::GetInstance()->m_Offset[14], &err);
		else if(id == 17)
			MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteDWord(id, PRO54::P_GOAL_POSITION_LL,  outValue[1] + MotionManager::GetInstance()->m_Offset[16], &err);
		else if(id == 19)
			MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteDWord(id, PRO54::P_GOAL_POSITION_LL,  outValue[2] + MotionManager::GetInstance()->m_Offset[18], &err);
		else if(id == 21)
			MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteDWord(id, PRO54::P_GOAL_POSITION_LL,  outValue[3] + MotionManager::GetInstance()->m_Offset[20], &err);
		else if(id == 23)
			MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteDWord(id, PRO54::P_GOAL_POSITION_LL,  outValue[4] + MotionManager::GetInstance()->m_Offset[22], &err);
		else if(id == 25)
			MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteDWord(id, PRO54::P_GOAL_POSITION_LL,  outValue[5] + MotionManager::GetInstance()->m_Offset[24], &err);

		else if(id == 16)
			MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteDWord(id, PRO54::P_GOAL_POSITION_LL,  outValue[6] + MotionManager::GetInstance()->m_Offset[15], &err);
		else if(id == 18)
			MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteDWord(id, PRO54::P_GOAL_POSITION_LL,  outValue[7] + MotionManager::GetInstance()->m_Offset[17], &err);
		else if(id == 20)
			MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteDWord(id, PRO54::P_GOAL_POSITION_LL,  outValue[8] + MotionManager::GetInstance()->m_Offset[19], &err);
		else if(id == 22)
			MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteDWord(id, PRO54::P_GOAL_POSITION_LL,  outValue[9] + MotionManager::GetInstance()->m_Offset[21], &err);
		else if(id == 24)
			MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteDWord(id, PRO54::P_GOAL_POSITION_LL,  outValue[10] + MotionManager::GetInstance()->m_Offset[23], &err);
		else if(id == 26)
			MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteDWord(id, PRO54::P_GOAL_POSITION_LL,  outValue[11] + MotionManager::GetInstance()->m_Offset[25], &err);



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


	RecursiveWalking::GetInstance()->BALANCE_ENABLE = true;
	RecursiveWalking::GetInstance()->DEBUG_PRINT = false;

	RecursiveWalking::GetInstance()->HIP_PITCH_OFFSET = 8.0;//6.0
	RecursiveWalking::GetInstance()->Initialize();

	RecursiveWalking::GetInstance()->WALK_STABILIZER_GAIN_RATIO = 2.0;
	RecursiveWalking::GetInstance()->IMU_GYRO_GAIN_RATIO = 0.0;//7.31*0.01;
	RecursiveWalking::GetInstance()->FORCE_MOMENT_DISTRIBUTION_RATIO = 0.4;

	RecursiveWalking::GetInstance()->BALANCE_X_GAIN     = +20.30*0.5*(RecursiveWalking::GetInstance()->FORCE_MOMENT_DISTRIBUTION_RATIO)*RecursiveWalking::GetInstance()->WALK_STABILIZER_GAIN_RATIO;
	RecursiveWalking::GetInstance()->BALANCE_Y_GAIN     = -20.30*(RecursiveWalking::GetInstance()->FORCE_MOMENT_DISTRIBUTION_RATIO)*RecursiveWalking::GetInstance()->WALK_STABILIZER_GAIN_RATIO;
	RecursiveWalking::GetInstance()->BALANCE_PITCH_GAIN = -0.06*0.5*(1-RecursiveWalking::GetInstance()->FORCE_MOMENT_DISTRIBUTION_RATIO)*RecursiveWalking::GetInstance()->WALK_STABILIZER_GAIN_RATIO;
	RecursiveWalking::GetInstance()->BALANCE_ROLL_GAIN  = -0.10*(1-RecursiveWalking::GetInstance()->FORCE_MOMENT_DISTRIBUTION_RATIO)*RecursiveWalking::GetInstance()->WALK_STABILIZER_GAIN_RATIO;

	RecursiveWalking::GetInstance()->FOOT_LANDING_OFFSET_GAIN =   +1.0*0;
	RecursiveWalking::GetInstance()->FOOT_LANDING_DETECT_N = 50;

	RecursiveWalking::GetInstance()->SYSTEM_CONTROL_UNIT_TIME_SEC = MotionModule::TIME_UNIT / 1000.0;
	RecursiveWalking::GetInstance()->FOOT_LANDING_DETECTION_TIME_MAX_SEC = 10.0;

	RecursiveWalking::GetInstance()->FOOT_ROLL_ADJUSTMENT_ABS_MAX_RAD = 10.0*PI/180;
	RecursiveWalking::GetInstance()->FOOT_PITCH_ADJUSTMENT_ABS_MAX_RAD = 10.0*PI/180;

	RecursiveWalking::GetInstance()->COB_X_ADJUSTMENT_ABS_MAX_MM = 50;
	RecursiveWalking::GetInstance()->COB_Y_ADJUSTMENT_ABS_MAX_MM = 50;


	RecursiveWalking::GetInstance()->P_GAIN = 64;
	RecursiveWalking::GetInstance()->I_GAIN = 0;
	RecursiveWalking::GetInstance()->D_GAIN = 0;



	////AddModule
	MotionManager::GetInstance()->AddModule((MotionModule*)RecursiveWalking::GetInstance());


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
	RecursiveWalking::GetInstance()->SetInitAngleinRad(RollInitAngleRad/((double)InitAngleWindowSize), PitchInitAngleRad/((double)InitAngleWindowSize));

	printf("Setting Init Angle is completed %f %f!\n", RollInitAngleRad/((double)InitAngleWindowSize),  PitchInitAngleRad/((double)InitAngleWindowSize));

	SetTheMotionEnableList();

	double period = 850*1.2;
	double setime = 3000;
	double dsp = 0.2*1.0;

	StepData stp0, stp1, stp2, stp3, stp4, stp5, stp6, stp7, stp8, stp9;
	stp0.PositionData.bMovingFoot = NFootMove;        stp0.PositionData.dFootHeight = 0.0;               stp0.PositionData.dZ_Swap_Amplitude = 0.0;
	stp0.PositionData.dShoulderSwingGain = 0.05;      stp0.PositionData.dElbowSwingGain = 0.1;
	stp0.PositionData.stRightFootPosition.x = 0.0;    stp0.PositionData.stRightFootPosition.y = -95.0;   stp0.PositionData.stRightFootPosition.z = 0.0;
	stp0.PositionData.stRightFootPosition.roll = 0.0; stp0.PositionData.stRightFootPosition.pitch = 0.0; stp0.PositionData.stRightFootPosition.yaw = 0.0;
	stp0.PositionData.stBodyPosition.z = 670.0;
	stp0.PositionData.stBodyPosition.roll = 0.0;      stp0.PositionData.stBodyPosition.pitch = 0.0;      stp0.PositionData.stBodyPosition.yaw = 0.0;
	stp0.PositionData.stLeftFootPosition.x = 0.0;     stp0.PositionData.stLeftFootPosition.y = 95.0;     stp0.PositionData.stLeftFootPosition.z = 0.0;
	stp0.PositionData.stLeftFootPosition.roll = 0.0;  stp0.PositionData.stLeftFootPosition.pitch = 0.0;  stp0.PositionData.stLeftFootPosition.yaw = 0.0;

	stp0.TimeData.bWalkingState = InWalkingStarting;
	stp0.TimeData.dAbsStepTime = setime;    stp0.TimeData.dDSPratio = dsp;
	stp0.TimeData.sigmoid_ratio_x = 1.0;    stp0.TimeData.sigmoid_ratio_y = 1.0;     stp0.TimeData.sigmoid_ratio_z = 1.0;
	stp0.TimeData.sigmoid_ratio_roll = 1.0; stp0.TimeData.sigmoid_ratio_pitch = 1.0; stp0.TimeData.sigmoid_ratio_yaw = 1.0;
	stp0.TimeData.sigmoid_distortion_x = 1.0;    stp0.TimeData.sigmoid_distortion_y = 1.0;     stp0.TimeData.sigmoid_distortion_z = 1.0;
	stp0.TimeData.sigmoid_distortion_roll = 1.0; stp0.TimeData.sigmoid_distortion_pitch = 1.0; stp0.TimeData.sigmoid_distortion_yaw = 1.0;



	stp1.PositionData.bMovingFoot = RFootMove;        stp1.PositionData.dFootHeight = 80.0;               stp1.PositionData.dZ_Swap_Amplitude = 10.0;
	stp1.PositionData.dShoulderSwingGain = 0.05;      stp1.PositionData.dElbowSwingGain = 0.1;
	stp1.PositionData.stRightFootPosition.x = 270.0;  stp1.PositionData.stRightFootPosition.y = -95.0;   stp1.PositionData.stRightFootPosition.z = (270.0-129.0)*tan(12.5*ANG2RAD);
	stp1.PositionData.stRightFootPosition.roll = 0.0; stp1.PositionData.stRightFootPosition.pitch = -12.5*ANG2RAD; stp1.PositionData.stRightFootPosition.yaw = 0.0;
	stp1.PositionData.stBodyPosition.z = 670.0;
	stp1.PositionData.stBodyPosition.roll = 0.0;      stp1.PositionData.stBodyPosition.pitch = 0.0;      stp1.PositionData.stBodyPosition.yaw = 0.0;
	stp1.PositionData.stLeftFootPosition.x = 0.0;     stp1.PositionData.stLeftFootPosition.y = 95.0;     stp1.PositionData.stLeftFootPosition.z = 0.0;
	stp1.PositionData.stLeftFootPosition.roll = 0.0;  stp1.PositionData.stLeftFootPosition.pitch = 0.0;  stp1.PositionData.stLeftFootPosition.yaw = 0.0;

	stp1.TimeData.bWalkingState = InWalking;
	stp1.TimeData.dAbsStepTime = setime + period*1;    stp1.TimeData.dDSPratio = dsp;
	stp1.TimeData.sigmoid_ratio_x = 1.0;    stp1.TimeData.sigmoid_ratio_y = 1.0;     stp1.TimeData.sigmoid_ratio_z = 1.0;
	stp1.TimeData.sigmoid_ratio_roll = 1.0; stp1.TimeData.sigmoid_ratio_pitch = 1.0; stp1.TimeData.sigmoid_ratio_yaw = 1.0;
	stp1.TimeData.sigmoid_distortion_x = 1.0;    stp1.TimeData.sigmoid_distortion_y = 1.0;     stp1.TimeData.sigmoid_distortion_z = 1.0;
	stp1.TimeData.sigmoid_distortion_roll = 1.0; stp1.TimeData.sigmoid_distortion_pitch = 1.0; stp1.TimeData.sigmoid_distortion_yaw = 1.0;



	stp2.PositionData.bMovingFoot = LFootMove;        stp2.PositionData.dFootHeight = 80.0;               stp2.PositionData.dZ_Swap_Amplitude = 10.0;
	stp2.PositionData.dShoulderSwingGain = 0.05;      stp2.PositionData.dElbowSwingGain = 0.1;
	stp2.PositionData.stRightFootPosition.x = 270.0;  stp2.PositionData.stRightFootPosition.y = -95.0;   stp2.PositionData.stRightFootPosition.z = (270.0-129.0)*tan(12.5*ANG2RAD);
	stp2.PositionData.stRightFootPosition.roll = 0.0; stp2.PositionData.stRightFootPosition.pitch = -12.5*ANG2RAD; stp2.PositionData.stRightFootPosition.yaw = 0.0;
	stp2.PositionData.stBodyPosition.z = 670.0 + (270.0-129.0)*tan(12.5*ANG2RAD);
	stp2.PositionData.stBodyPosition.roll = 0.0;      stp2.PositionData.stBodyPosition.pitch = 0.0;      stp2.PositionData.stBodyPosition.yaw = 0.0;
	stp2.PositionData.stLeftFootPosition.x = 410.0;     stp2.PositionData.stLeftFootPosition.y = 95.0;   stp2.PositionData.stLeftFootPosition.z = (410.0-129.0)*tan(12.5*ANG2RAD);
	stp2.PositionData.stLeftFootPosition.roll = 0.0;  stp2.PositionData.stLeftFootPosition.pitch = -12.5*ANG2RAD;  stp2.PositionData.stLeftFootPosition.yaw = 0.0;

	stp2.TimeData.bWalkingState = InWalking;
	stp2.TimeData.dAbsStepTime = setime + period*2;    stp2.TimeData.dDSPratio = dsp;
	stp2.TimeData.sigmoid_ratio_x = 1.0;    stp2.TimeData.sigmoid_ratio_y = 1.0;     stp2.TimeData.sigmoid_ratio_z = 1.0;
	stp2.TimeData.sigmoid_ratio_roll = 1.0; stp2.TimeData.sigmoid_ratio_pitch = 1.0; stp2.TimeData.sigmoid_ratio_yaw = 1.0;
	stp2.TimeData.sigmoid_distortion_x = 1.0;    stp2.TimeData.sigmoid_distortion_y = 1.0;     stp2.TimeData.sigmoid_distortion_z = 1.0;
	stp2.TimeData.sigmoid_distortion_roll = 1.0; stp2.TimeData.sigmoid_distortion_pitch = 1.0; stp2.TimeData.sigmoid_distortion_yaw = 1.0;



	stp3.PositionData.bMovingFoot = RFootMove;        stp3.PositionData.dFootHeight = 80.0;               stp3.PositionData.dZ_Swap_Amplitude = 10.0;
	stp3.PositionData.dShoulderSwingGain = 0.05;      stp3.PositionData.dElbowSwingGain = 0.1;
	stp3.PositionData.stRightFootPosition.x = 570.0;  stp3.PositionData.stRightFootPosition.y = -95.0;   stp3.PositionData.stRightFootPosition.z = (570.0-129.0)*tan(12.5*ANG2RAD);
	stp3.PositionData.stRightFootPosition.roll = 0.0; stp3.PositionData.stRightFootPosition.pitch = -12.5*ANG2RAD; stp3.PositionData.stRightFootPosition.yaw = 0.0;
	stp3.PositionData.stBodyPosition.z = 670.0+(410.0-129.0)*tan(12.5*ANG2RAD);
	stp3.PositionData.stBodyPosition.roll = 0.0;      stp3.PositionData.stBodyPosition.pitch = 0.0;      stp3.PositionData.stBodyPosition.yaw = 0.0;
	stp3.PositionData.stLeftFootPosition.x = 410.0;     stp3.PositionData.stLeftFootPosition.y = 95.0;   stp3.PositionData.stLeftFootPosition.z = (410.0-129.0)*tan(12.5*ANG2RAD);
	stp3.PositionData.stLeftFootPosition.roll = 0.0;  stp3.PositionData.stLeftFootPosition.pitch = -12.5*ANG2RAD;  stp3.PositionData.stLeftFootPosition.yaw = 0.0;

	stp3.TimeData.bWalkingState = InWalking;
	stp3.TimeData.dAbsStepTime = setime + period*3;    stp3.TimeData.dDSPratio = dsp;
	stp3.TimeData.sigmoid_ratio_x = 1.0;    stp3.TimeData.sigmoid_ratio_y = 1.0;     stp3.TimeData.sigmoid_ratio_z = 1.0;
	stp3.TimeData.sigmoid_ratio_roll = 1.0; stp3.TimeData.sigmoid_ratio_pitch = 1.0; stp3.TimeData.sigmoid_ratio_yaw = 1.0;
	stp3.TimeData.sigmoid_distortion_x = 1.0;    stp3.TimeData.sigmoid_distortion_y = 1.0;     stp3.TimeData.sigmoid_distortion_z = 1.0;
	stp3.TimeData.sigmoid_distortion_roll = 1.0; stp3.TimeData.sigmoid_distortion_pitch = 1.0; stp3.TimeData.sigmoid_distortion_yaw = 1.0;



	stp4.PositionData.bMovingFoot = LFootMove;        stp4.PositionData.dFootHeight = 80.0;               stp4.PositionData.dZ_Swap_Amplitude = 10.0;
	stp4.PositionData.dShoulderSwingGain = 0.05;      stp4.PositionData.dElbowSwingGain = 0.1;
	stp4.PositionData.stRightFootPosition.x = 570.0;  stp4.PositionData.stRightFootPosition.y = -95.0;   stp4.PositionData.stRightFootPosition.z = (570.0-129.0)*tan(12.5*ANG2RAD);
	stp4.PositionData.stRightFootPosition.roll = 0.0; stp4.PositionData.stRightFootPosition.pitch = -12.5*ANG2RAD; stp4.PositionData.stRightFootPosition.yaw = 0.0;
	stp4.PositionData.stBodyPosition.z = 670.0+(570.0-129.0)*tan(12.5*ANG2RAD);
	stp4.PositionData.stBodyPosition.roll = 0.0;      stp4.PositionData.stBodyPosition.pitch = 0.0;      stp4.PositionData.stBodyPosition.yaw = 0.0;
	stp4.PositionData.stLeftFootPosition.x = 570.0;   stp4.PositionData.stLeftFootPosition.y = 95.0;     stp4.PositionData.stLeftFootPosition.z = (570.0-129.0)*tan(12.5*ANG2RAD);
	stp4.PositionData.stLeftFootPosition.roll = 0.0;  stp4.PositionData.stLeftFootPosition.pitch = -12.5*ANG2RAD;  stp4.PositionData.stLeftFootPosition.yaw = 0.0;

	stp4.TimeData.bWalkingState = InWalking;
	stp4.TimeData.dAbsStepTime = setime + period*4;    stp4.TimeData.dDSPratio = dsp;
	stp4.TimeData.sigmoid_ratio_x = 1.0;    stp4.TimeData.sigmoid_ratio_y = 1.0;     stp4.TimeData.sigmoid_ratio_z = 1.0;
	stp4.TimeData.sigmoid_ratio_roll = 1.0; stp4.TimeData.sigmoid_ratio_pitch = 1.0; stp4.TimeData.sigmoid_ratio_yaw = 1.0;
	stp4.TimeData.sigmoid_distortion_x = 1.0;    stp4.TimeData.sigmoid_distortion_y = 1.0;     stp4.TimeData.sigmoid_distortion_z = 1.0;
	stp4.TimeData.sigmoid_distortion_roll = 1.0; stp4.TimeData.sigmoid_distortion_pitch = 1.0; stp4.TimeData.sigmoid_distortion_yaw = 1.0;



	stp5.PositionData.bMovingFoot = RFootMove;        stp5.PositionData.dFootHeight = 80.0;               stp5.PositionData.dZ_Swap_Amplitude = 10.0;
	stp5.PositionData.dShoulderSwingGain = 0.05;      stp5.PositionData.dElbowSwingGain = 0.1;
	stp5.PositionData.stRightFootPosition.x = 830.0;  stp5.PositionData.stRightFootPosition.y = -95.0;   stp5.PositionData.stRightFootPosition.z = (570.0-129.0)*tan(12.5*ANG2RAD);
	stp5.PositionData.stRightFootPosition.roll = 0.0; stp5.PositionData.stRightFootPosition.pitch = 12.5*ANG2RAD; stp5.PositionData.stRightFootPosition.yaw = 0.0;
	stp5.PositionData.stBodyPosition.z = 660.0+(570.0-129.0)*tan(12.5*ANG2RAD);
	stp5.PositionData.stBodyPosition.roll = 0.0;      stp5.PositionData.stBodyPosition.pitch = 0.0;      stp5.PositionData.stBodyPosition.yaw = 0.0;
	stp5.PositionData.stLeftFootPosition.x = 570.0;   stp5.PositionData.stLeftFootPosition.y = 95.0;     stp5.PositionData.stLeftFootPosition.z = (570.0-129.0)*tan(12.5*ANG2RAD);
	stp5.PositionData.stLeftFootPosition.roll = 0.0;  stp5.PositionData.stLeftFootPosition.pitch = -12.5*ANG2RAD;  stp5.PositionData.stLeftFootPosition.yaw = 0.0;

	stp5.TimeData.bWalkingState = InWalking;
	stp5.TimeData.dAbsStepTime = setime + period*5;    stp5.TimeData.dDSPratio = dsp;
	stp5.TimeData.sigmoid_ratio_x = 1.0;    stp5.TimeData.sigmoid_ratio_y = 1.0;     stp5.TimeData.sigmoid_ratio_z = 1.0;
	stp5.TimeData.sigmoid_ratio_roll = 1.0; stp5.TimeData.sigmoid_ratio_pitch = 1.0; stp5.TimeData.sigmoid_ratio_yaw = 1.0;
	stp5.TimeData.sigmoid_distortion_x = 1.0;    stp5.TimeData.sigmoid_distortion_y = 1.0;     stp5.TimeData.sigmoid_distortion_z = 1.0;
	stp5.TimeData.sigmoid_distortion_roll = 1.0; stp5.TimeData.sigmoid_distortion_pitch = 1.0; stp5.TimeData.sigmoid_distortion_yaw = 1.0;



	stp6.PositionData.bMovingFoot = LFootMove;        stp6.PositionData.dFootHeight = 80.0;               stp6.PositionData.dZ_Swap_Amplitude = 10.0;
	stp6.PositionData.dShoulderSwingGain = 0.05;      stp6.PositionData.dElbowSwingGain = 0.1;
	stp6.PositionData.stRightFootPosition.x = 830.0;  stp6.PositionData.stRightFootPosition.y = -95.0;   stp6.PositionData.stRightFootPosition.z = (570.0-129.0)*tan(12.5*ANG2RAD);
	stp6.PositionData.stRightFootPosition.roll = 0.0; stp6.PositionData.stRightFootPosition.pitch = 12.5*ANG2RAD; stp6.PositionData.stRightFootPosition.yaw = 0.0;
	stp6.PositionData.stBodyPosition.z = 650.0+(570.0-129.0)*tan(12.5*ANG2RAD);
	stp6.PositionData.stBodyPosition.roll = 0.0;      stp6.PositionData.stBodyPosition.pitch = 0.0;      stp6.PositionData.stBodyPosition.yaw = 0.0;
	stp6.PositionData.stLeftFootPosition.x = 830.0;   stp6.PositionData.stLeftFootPosition.y = 95.0;     stp6.PositionData.stLeftFootPosition.z = (570.0-129.0)*tan(12.5*ANG2RAD);
	stp6.PositionData.stLeftFootPosition.roll = 0.0;  stp6.PositionData.stLeftFootPosition.pitch = 12.5*ANG2RAD;  stp6.PositionData.stLeftFootPosition.yaw = 0.0;

	stp6.TimeData.bWalkingState = InWalking;
	stp6.TimeData.dAbsStepTime = setime + period*6;    stp6.TimeData.dDSPratio = dsp;
	stp6.TimeData.sigmoid_ratio_x = 1.0;    stp6.TimeData.sigmoid_ratio_y = 1.0;     stp6.TimeData.sigmoid_ratio_z = 1.0;
	stp6.TimeData.sigmoid_ratio_roll = 1.0; stp6.TimeData.sigmoid_ratio_pitch = 1.0; stp6.TimeData.sigmoid_ratio_yaw = 1.0;
	stp6.TimeData.sigmoid_distortion_x = 1.0;    stp6.TimeData.sigmoid_distortion_y = 1.0;     stp6.TimeData.sigmoid_distortion_z = 1.0;
	stp6.TimeData.sigmoid_distortion_roll = 1.0; stp6.TimeData.sigmoid_distortion_pitch = 1.0; stp6.TimeData.sigmoid_distortion_yaw = 1.0;


	stp7.PositionData.bMovingFoot = RFootMove;        stp7.PositionData.dFootHeight = 80.0;               stp7.PositionData.dZ_Swap_Amplitude = 10.0;
	stp7.PositionData.dShoulderSwingGain = 0.05;      stp7.PositionData.dElbowSwingGain = 0.1;
	stp7.PositionData.stRightFootPosition.x = 970.0;  stp7.PositionData.stRightFootPosition.y = -95.0;   stp7.PositionData.stRightFootPosition.z = (570.0-129.0)*tan(12.5*ANG2RAD) - 140.0*tan(12.5*ANG2RAD);
	stp7.PositionData.stRightFootPosition.roll = 0.0; stp7.PositionData.stRightFootPosition.pitch = 12.5*ANG2RAD; stp7.PositionData.stRightFootPosition.yaw = 0.0;
	stp7.PositionData.stBodyPosition.z = 650.0+(570.0-129.0)*tan(12.5*ANG2RAD) - 140.0*tan(12.5*ANG2RAD);
	stp7.PositionData.stBodyPosition.roll = 0.0;      stp7.PositionData.stBodyPosition.pitch = 0.0;      stp7.PositionData.stBodyPosition.yaw = 0.0;
	stp7.PositionData.stLeftFootPosition.x = 830.0;   stp7.PositionData.stLeftFootPosition.y = 95.0;     stp7.PositionData.stLeftFootPosition.z = (570.0-129.0)*tan(12.5*ANG2RAD);
	stp7.PositionData.stLeftFootPosition.roll = 0.0;  stp7.PositionData.stLeftFootPosition.pitch = 12.5*ANG2RAD;  stp7.PositionData.stLeftFootPosition.yaw = 0.0;

	stp7.TimeData.bWalkingState = InWalking;
	stp7.TimeData.dAbsStepTime = setime + period*7;    stp7.TimeData.dDSPratio = dsp;
	stp7.TimeData.sigmoid_ratio_x = 1.0;    stp7.TimeData.sigmoid_ratio_y = 1.0;     stp7.TimeData.sigmoid_ratio_z = 1.0;
	stp7.TimeData.sigmoid_ratio_roll = 1.0; stp7.TimeData.sigmoid_ratio_pitch = 1.0; stp7.TimeData.sigmoid_ratio_yaw = 1.0;
	stp7.TimeData.sigmoid_distortion_x = 1.0;    stp7.TimeData.sigmoid_distortion_y = 1.0;     stp7.TimeData.sigmoid_distortion_z = 1.0;
	stp7.TimeData.sigmoid_distortion_roll = 1.0; stp7.TimeData.sigmoid_distortion_pitch = 1.0; stp7.TimeData.sigmoid_distortion_yaw = 1.0;




	stp8.PositionData.bMovingFoot = LFootMove;        stp8.PositionData.dFootHeight = 80.0;               stp8.PositionData.dZ_Swap_Amplitude = 10.0;
	stp8.PositionData.dShoulderSwingGain = 0.05;      stp8.PositionData.dElbowSwingGain = 0.1;
	stp8.PositionData.stRightFootPosition.x = 970.0;  stp8.PositionData.stRightFootPosition.y = -95.0;   stp8.PositionData.stRightFootPosition.z = (570.0-129.0)*tan(12.5*ANG2RAD) - 140.0*tan(12.5*ANG2RAD);
	stp8.PositionData.stRightFootPosition.roll = 0.0; stp8.PositionData.stRightFootPosition.pitch = 12.5*ANG2RAD; stp8.PositionData.stRightFootPosition.yaw = 0.0;
	stp8.PositionData.stBodyPosition.z = 640.0+(570.0-129.0)*tan(12.5*ANG2RAD) - 140.0*tan(12.5*ANG2RAD);
	stp8.PositionData.stBodyPosition.roll = 0.0;      stp8.PositionData.stBodyPosition.pitch = 0.0;      stp8.PositionData.stBodyPosition.yaw = 0.0;
	stp8.PositionData.stLeftFootPosition.x = 970.0;   stp8.PositionData.stLeftFootPosition.y = 95.0;     stp8.PositionData.stLeftFootPosition.z = (570.0-129.0)*tan(12.5*ANG2RAD) - 140.0*tan(12.5*ANG2RAD);
	stp8.PositionData.stLeftFootPosition.roll = 0.0;  stp8.PositionData.stLeftFootPosition.pitch = 12.5*ANG2RAD;  stp8.PositionData.stLeftFootPosition.yaw = 0.0;

	stp8.TimeData.bWalkingState = InWalking;
	stp8.TimeData.dAbsStepTime = setime + period*8;    stp8.TimeData.dDSPratio = dsp;
	stp8.TimeData.sigmoid_ratio_x = 1.0;    stp8.TimeData.sigmoid_ratio_y = 1.0;     stp7.TimeData.sigmoid_ratio_z = 1.0;
	stp8.TimeData.sigmoid_ratio_roll = 1.0; stp8.TimeData.sigmoid_ratio_pitch = 1.0; stp7.TimeData.sigmoid_ratio_yaw = 1.0;
	stp8.TimeData.sigmoid_distortion_x = 1.0;    stp8.TimeData.sigmoid_distortion_y = 1.0;     stp7.TimeData.sigmoid_distortion_z = 1.0;
	stp8.TimeData.sigmoid_distortion_roll = 1.0; stp8.TimeData.sigmoid_distortion_pitch = 1.0; stp7.TimeData.sigmoid_distortion_yaw = 1.0;



	stp9.PositionData.bMovingFoot = NFootMove;        stp9.PositionData.dFootHeight = 0.0;               stp9.PositionData.dZ_Swap_Amplitude = 0.0;
	stp9.PositionData.dShoulderSwingGain = 0.05;      stp9.PositionData.dElbowSwingGain = 0.1;
	stp9.PositionData.stRightFootPosition.x = 970.0;  stp9.PositionData.stRightFootPosition.y = -95.0;   stp9.PositionData.stRightFootPosition.z = (570.0-129.0)*tan(12.5*ANG2RAD) - 140.0*tan(12.5*ANG2RAD);
	stp9.PositionData.stRightFootPosition.roll = 0.0; stp9.PositionData.stRightFootPosition.pitch = 12.5*ANG2RAD; stp9.PositionData.stRightFootPosition.yaw = 0.0;
	stp9.PositionData.stBodyPosition.z = 640.0+(570.0-129.0)*tan(12.5*ANG2RAD) - 140.0*tan(12.5*ANG2RAD);
	stp9.PositionData.stBodyPosition.roll = 0.0;      stp9.PositionData.stBodyPosition.pitch = 0.0;      stp9.PositionData.stBodyPosition.yaw = 0.0;
	stp9.PositionData.stLeftFootPosition.x = 970.0;   stp9.PositionData.stLeftFootPosition.y = 95.0;     stp9.PositionData.stLeftFootPosition.z = (570.0-129.0)*tan(12.5*ANG2RAD) - 140.0*tan(12.5*ANG2RAD);
	stp9.PositionData.stLeftFootPosition.roll = 0.0;  stp9.PositionData.stLeftFootPosition.pitch = 12.5*ANG2RAD;  stp9.PositionData.stLeftFootPosition.yaw = 0.0;

	stp9.TimeData.bWalkingState = InWalkingEnding;
	stp9.TimeData.dAbsStepTime = setime + period*8 + setime;    stp9.TimeData.dDSPratio = dsp;
	stp9.TimeData.sigmoid_ratio_x = 1.0;    stp9.TimeData.sigmoid_ratio_y = 1.0;     stp9.TimeData.sigmoid_ratio_z = 1.0;
	stp9.TimeData.sigmoid_ratio_roll = 1.0; stp9.TimeData.sigmoid_ratio_pitch = 1.0; stp9.TimeData.sigmoid_ratio_yaw = 1.0;
	stp9.TimeData.sigmoid_distortion_x = 1.0;    stp9.TimeData.sigmoid_distortion_y = 1.0;     stp9.TimeData.sigmoid_distortion_z = 1.0;
	stp9.TimeData.sigmoid_distortion_roll = 1.0; stp9.TimeData.sigmoid_distortion_pitch = 1.0; stp9.TimeData.sigmoid_distortion_yaw = 1.0;

//	stp6.PositionData.bMovingFoot = NFootMove;        stp6.PositionData.dFootHeight = 0.0;               stp6.PositionData.dZ_Swap_Amplitude = 0.0;
//	stp6.PositionData.dShoulderSwingGain = 0.05;      stp6.PositionData.dElbowSwingGain = 0.1;
//	stp6.PositionData.stRightFootPosition.x = 500.0;  stp6.PositionData.stRightFootPosition.y = -95.0;   stp6.PositionData.stRightFootPosition.z = (270.0-129.0)*tan(12.5*ANG2RAD);
//	stp6.PositionData.stRightFootPosition.roll = 0.0; stp6.PositionData.stRightFootPosition.pitch = -12.5*ANG2RAD; stp4.PositionData.stRightFootPosition.yaw = 0.0;
//	stp6.PositionData.stBodyPosition.z = 700.0;
//	stp6.PositionData.stBodyPosition.roll = 0.0;      stp6.PositionData.stBodyPosition.pitch = 0.0;      stp6.PositionData.stBodyPosition.yaw = 0.0;
//	stp6.PositionData.stLeftFootPosition.x = 500.0;   stp6.PositionData.stLeftFootPosition.y = 95.0;     stp6.PositionData.stLeftFootPosition.z = (270.0-129.0)*tan(12.5*ANG2RAD);
//	stp6.PositionData.stLeftFootPosition.roll = 0.0;  stp6.PositionData.stLeftFootPosition.pitch =-12.5*ANG2RAD;  stp6.PositionData.stLeftFootPosition.yaw = 0.0;
//
//	stp6.TimeData.bWalkingState = InWalking;
//	stp6.TimeData.dAbsStepTime = 11200.0;    stp6.TimeData.dDSPratio = 0.2;
//	stp6.TimeData.sigmoid_ratio_x = 1.0;    stp6.TimeData.sigmoid_ratio_y = 1.0;     stp6.TimeData.sigmoid_ratio_z = 1.0;
//	stp6.TimeData.sigmoid_ratio_roll = 1.0; stp6.TimeData.sigmoid_ratio_pitch = 1.0; stp6.TimeData.sigmoid_ratio_yaw = 1.0;
//	stp6.TimeData.sigmoid_distortion_x = 1.0;    stp6.TimeData.sigmoid_distortion_y = 1.0;     stp6.TimeData.sigmoid_distortion_z = 1.0;
//	stp6.TimeData.sigmoid_distortion_roll = 1.0; stp6.TimeData.sigmoid_distortion_pitch = 1.0; stp6.TimeData.sigmoid_distortion_yaw = 1.0;


	RecursiveWalking::GetInstance()->AddStepData(stp0);
	RecursiveWalking::GetInstance()->AddStepData(stp1);
	RecursiveWalking::GetInstance()->AddStepData(stp2);
	RecursiveWalking::GetInstance()->AddStepData(stp3);
	RecursiveWalking::GetInstance()->AddStepData(stp4);
	RecursiveWalking::GetInstance()->AddStepData(stp5);
	RecursiveWalking::GetInstance()->AddStepData(stp6);
	RecursiveWalking::GetInstance()->AddStepData(stp7);
	RecursiveWalking::GetInstance()->AddStepData(stp8);
	RecursiveWalking::GetInstance()->AddStepData(stp9);
	if(RecursiveWalking::GetInstance()->CalcWalkingPattern() == false)
		return 0;


	printf("press any key to start");
	_getch();
	RecursiveWalking::GetInstance()->Start();
	MotionManager::GetInstance()->StartTimer();
	while(true)
	{
		if(kbhit())
			if(_getch() == 27)
				break;

		if(RecursiveWalking::GetInstance()->IsRunning() == false)
		{
			if(_getch() != 27)
			{
				printf("restart\n");
				RecursiveWalking::GetInstance()->Start();
			}
			else
				break;
		}
		usleep(8000);
	}
}


