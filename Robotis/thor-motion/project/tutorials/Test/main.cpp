/*
 * main.cpp
 *
 *  Created on: 2013. 1. 24.
 *      Author: hjsong
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <termios.h>
#include <term.h>
#include <fcntl.h>
#include <ncurses.h>


#include "motion/modules/dynamic_walking.h"
#include "motion/PRO54.h"
#include "motion/PRO42.h"
#include "motion/motionstatus.h"
#include "motion/motionmanager.h"
#include "motion/dxl_comm.h"
#include "sensor/ins/ins.h"


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
	if(MotionManager::GetInstance()->Initialize() == false)
	{
		printf("Fail to initialize Motion Manager!\n");
		return 0;
	}

	for(unsigned int index = 0; index < MotionStatus::m_CurrentJoints.size(); index++)
	{
		int id = MotionStatus::m_CurrentJoints[index].m_ID;

		int err, val;
		MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteDWord(id, PRO54::P_GOAL_ACCELATION_LL, 0, &err);
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
	DynamicWalking::GetInstance()->Initialize();
	DynamicWalking::GetInstance()->BALANCE_ENABLE = true;
	DynamicWalking::GetInstance()->BALANCE_KNEE_GAIN = 0.3*1.0;
	DynamicWalking::GetInstance()->BALANCE_ANKLE_PITCH_GAIN = 0.9*1.0;
	DynamicWalking::GetInstance()->BALANCE_HIP_ROLL_GAIN = 0.5*2.0/2.0;
	DynamicWalking::GetInstance()->BALANCE_ANKLE_ROLL_GAIN = 1.0*2.0/2.0;

	DynamicWalking::GetInstance()->HIP_PITCH_OFFSET = 6.0;//8.0

	DynamicWalking::GetInstance()->P_GAIN = 64;
	DynamicWalking::GetInstance()->I_GAIN = 0;
	DynamicWalking::GetInstance()->D_GAIN = 0;
	fprintf(stderr, " Start3\n");
	MotionManager::GetInstance()->AddModule((MotionModule*)DynamicWalking::GetInstance());
	fprintf(stderr, " Start4\n");
	printf("Start Dynamic Walking Test!\n");
	_getch();

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

	while(true)
	{
		if(kbhit())
			if(_getch() == 27)
			{
				break;
			}
	}



}

//int main(void)
//{
//	unsigned char param[9];
//	param[0] = 7;
//	param[1] = DXL_LOBYTE(DXL_LOWORD(0));
//	param[2] = DXL_HIBYTE(DXL_LOWORD(0));
//	param[3] = DXL_LOBYTE(DXL_LOWORD(0));
//	param[4] = DXL_LOBYTE(DXL_LOWORD(0));
//	param[0] = 7;
//	param[1] = DXL_LOBYTE(DXL_LOWORD(0));
//	param[2] = DXL_HIBYTE(DXL_LOWORD(0));
//	param[3] = DXL_LOBYTE(DXL_LOWORD(0));
//	param[4] = DXL_LOBYTE(DXL_LOWORD(0));
//
//	Dynamixel DXL("/dev/ttyUSB1");
//	DXL.Connect();
//
//	char temp;
//	while(true)
//	{
//		printf("...\n");
//		temp = _getch();
//		if(temp == 27)
//			break;
//		param[0] = 30;
//		param[1] = DXL_LOBYTE(1900);
//		param[2] = DXL_HIBYTE(1900);
//		param[3] = 32;
//		param[4] = DXL_LOBYTE(1900);
//		param[5] = DXL_HIBYTE(1900);
//		param[6] = 34;
//		param[7] = DXL_LOBYTE(1900);
//		param[8] = DXL_HIBYTE(1900);
//		DXL.SyncWrite(30, 2, param, 9);
//
//		printf("...\n");
//		temp = _getch();
//		if(temp == 27)
//			break;
//		param[0] = 30;
//		param[1] = DXL_LOBYTE(2048);
//		param[2] = DXL_HIBYTE(2048);
//		param[3] = 32;
//		param[4] = DXL_LOBYTE(2048);
//		param[5] = DXL_HIBYTE(2048);
//		param[6] = 34;
//		param[7] = DXL_LOBYTE(2048);
//		param[8] = DXL_HIBYTE(2048);
//		DXL.SyncWrite(30, 2, param, 9);
//	}
//	DXL.Disconnect();


//	unsigned char param[9];
//	param[0] = 29;
//	param[1] = DXL_LOBYTE(1700);
//	param[2] = DXL_HIBYTE(1700);
//	param[3] = 31;
//	param[4] = DXL_LOBYTE(1700);
//	param[5] = DXL_HIBYTE(1700);
//	param[6] = 33;
//	param[7] = DXL_LOBYTE(1700);
//	param[8] = DXL_HIBYTE(1700);
//
//	Dynamixel DXL("/dev/ttyUSB1");
//	DXL.Connect();
//
//	char temp;
//	while(true)
//	{
//		printf("...\n");
//		temp = _getch();
//		if(temp == 27)
//			break;
//		param[0] = 30;
//		param[1] = DXL_LOBYTE(1900);
//		param[2] = DXL_HIBYTE(1900);
//		param[3] = 32;
//		param[4] = DXL_LOBYTE(1900);
//		param[5] = DXL_HIBYTE(1900);
//		param[6] = 34;
//		param[7] = DXL_LOBYTE(1900);
//		param[8] = DXL_HIBYTE(1900);
//		DXL.SyncWrite(30, 2, param, 9);
//
//		printf("...\n");
//		temp = _getch();
//		if(temp == 27)
//			break;
//		param[0] = 30;
//		param[1] = DXL_LOBYTE(2048);
//		param[2] = DXL_HIBYTE(2048);
//		param[3] = 32;
//		param[4] = DXL_LOBYTE(2048);
//		param[5] = DXL_HIBYTE(2048);
//		param[6] = 34;
//		param[7] = DXL_LOBYTE(2048);
//		param[8] = DXL_HIBYTE(2048);
//		DXL.SyncWrite(30, 2, param, 9);
//	}
//	DXL.Disconnect();

//	//measurement the calculating, writing, reading time
//		MotionManager* manager;
//		manager = MotionManager::GetInstance();
//
//		if(manager->Initialize() == false)
//		{
//			printf("Fail to initialize MotionManager\n");
//			return 0;
//		}
//
//		minIni* ini = new minIni("config.ini");
//		manager->LoadINISettings(ini);
//		//manager->CheckFTSensorSetting();
//		manager->AddModule((MotionModule*) Test::GetInstance());
//		printf("Press the ENTER key to begin!\n");
//		_getch();
//		//return 1;
//		printf("Start\n");
//		manager->StartTimer();
//
//		printf("\n");
//		//printf("????\n");
//
//		while(true)
//		{
//			if(kbhit())
//			{
//				if(_getch() == 27)
//					break;
//			}
//
//			usleep(3000);
//		}
//
//		manager->StopTimer();
//
//		return 1;

//			//BulkReadTest with MotionManager
//		MotionManager* manager;
//
//		manager = MotionManager::GetInstance();
//
//		if(manager->Initialize() == false)
//		{
//			printf("Fail to initialize MotionManager. bp fail\n");
//			return 1;
//		}
//
//		manager->AddModule((MotionModule*) Test::GetInstance());
//
//		printf("Press the ENTER key to begin!\n");
//		_getch();
//		//return 1;
//		printf("Start\n");
//		manager->StartTimer();
//
//		while(_getch() != 27);
//
//		manager->StopTimer();
//
//		return 1;
//
//



//	MotionManager* manager;
//	manager = MotionManager::GetInstance();
//
//	if(manager->Initialize() == false)
//	{
//		printf("Fail to initialize MotionManager\n");
//		return 0;
//	}
//
//	minIni* ini = new minIni("config.ini");
//	manager->LoadINISettings(ini);
//	manager->CheckFTSensorSetting();
//	manager->AddModule((MotionModule*) Test::GetInstance());
//	printf("Press the ENTER key to begin!\n");
//	_getch();
//	//return 1;
//	printf("Start\n");
//	manager->StartTimer();
//
//	printf("\n");
//	printf("????\n");
//
//	while(true)
//	{
//		if(kbhit())
//		{
//			if(_getch() == 27)
//				break;
//		}
//		//printf("??????\n");
////		printf("\r");
////		printf("%f %f %f %f %f %f",
////				MotionStatus::L_ARM_FX,
////				MotionStatus::L_ARM_FY,
////				MotionStatus::L_ARM_FZ,
////				MotionStatus::L_ARM_TX,
////				MotionStatus::L_ARM_TY,
////				MotionStatus::L_ARM_TZ );
//		//printf("\r");
//		//printf("%f\t %f\t %f\t %f\t %f\t %f\t ",
//		//		MotionStatus::R_ARM_FX,
//		//		MotionStatus::R_ARM_FY,
//		//		MotionStatus::R_ARM_FZ,
//		//		MotionStatus::R_ARM_TX,
//		//		MotionStatus::R_ARM_TY,
//		//		MotionStatus::R_ARM_TZ );
//		usleep(3000);
//	}
//
//	manager->StopTimer();
//
//	return 1;

	//	//BulkReadTest with MotionManager
//	MotionManager* manager;
//
//	manager = MotionManager::GetInstance();
//
//	if(manager->Initialize() == false)
//	{
//		printf("Fail to initialize MotionManager. bp fail\n");
//		return 1;
//	}
//
//	manager->AddModule((MotionModule*) Test::GetInstance());
//
//	printf("Press the ENTER key to begin!\n");
//	_getch();
//	//return 1;
//	printf("Start\n");
//	manager->StartTimer();
//
//	while(_getch() != 27);
//
//	manager->StopTimer();
//
//	return 1;


	// BulkReadTest
	//	Dynamixel dxl("/dev/ttyUSB0");
	//	BulkRead br(&dxl);
	//
	//	dxl.Connect();
	//
//	BulkReadData tempData;
//	tempData.iID = 20;
//	tempData.iStartAddr = 611;
//	tempData.iLength = 4;
//	std::vector<BulkReadData> m_Data;
//	m_Data.push_back(tempData);
//
//	m_Data[0].pucTable = new unsigned char[4];
//
//
//
//	//int result = dxl.BulkRead(m_Data);
//	br.AddBulkReadData(20, 611, 4);
//	int result = br.SendTxPacket();
//	printf("2222222 %d\n", result);
//	int val;
//	br.GetDwordValue(20, 611, (long*)&val);
//	printf("%d\n", val );
//
//	//long value = DXL_MAKEDWORD( DXL_MAKEWORD(m_Data[0].pucTable[0], m_Data[0].pucTable[1]) , DXL_MAKEWORD(m_Data[0].pucTable[2], m_Data[0].pucTable[3] )   );
//	//printf("%d\n", value);
//
//	dxl.Disconnect();

//    printf( "\n===== First Test Program for Thor =====\n\n");
////    int nn = 0;
////    clock_t startTime, endTime;
////
////    startTime = clock();
////    while(true)
////    {
////    	endTime = clock();
////
////    	if( ( (double)(endTime - startTime) ) / CLOCKS_PER_SEC > 3.0)
////    	{
////    		nn++;
////    		printf("3.0 elapsed   %d\n", nn);
////    		startTime = clock();
////    	}
////    }
//
//    MotionManager* manager;
//
//    manager = MotionManager::GetInstance();
//
//    manager->DEBUG_PRINT = true;
//
//    if(manager->Initialize() == false)
//    {
//    	printf("Fail to initialize MotionManager. bp fail\n");
//    	return 1;
//    }
//
//    if(manager->InitforTest() == false)
//    {
//    	printf("Fail to initialize vel & accel value.\n");
//    	return 1;
//    }
//
//    printf("2222\n");
//    for(int i = 0; i < 36 ; i++)
//    {
//    	MotionStatus::m_EnableList[i].uID = "Test";
//    	printf("en change %d.\n",i);
//    }
//    //return 1;
//    manager->SetEnable();
//    //return 1;
//    manager->AddModule((MotionModule*) Test::GetInstance());
//    //return 1;
//
//    printf("Press the ENTER key to begin!\n");
//    _getch();
//    //return 1;
//    printf("Start\n");
//    manager->StartTimer();
//
//    while(_getch() != 27);
//
//    manager->StopTimer();
//
//    return 1;

//    Action::GetInstance()->LoadFile(MOTION_FILE_PATH);
//
//    //////////////////// Framework Initialize ////////////////////////////
//    LinuxCM730 linux_cm730("/dev/ttyUSB0");
//    CM730 cm730(&linux_cm730);
//    if(MotionManager::GetInstance()->Initialize(&cm730) == false)
//    {
//        printf("Fail to initialize Motion Manager!\n");
//            return 0;
//    }
//    MotionManager::GetInstance()->LoadINISettings(ini);
//    MotionManager::GetInstance()->AddModule((MotionModule*)Action::GetInstance());
//    LinuxMotionTimer *motion_timer = new LinuxMotionTimer(MotionManager::GetInstance());
//    motion_timer->Start();
//    /////////////////////////////////////////////////////////////////////
//
//    MotionManager::GetInstance()->SetEnable(true);
//
//    Action::GetInstance()->Start(1);    /* Init(stand up) pose */
//    while(Action::GetInstance()->IsRunning()) usleep(8*1000);
//
//    printf("Press the ENTER key to begin!\n");
//    getchar();
//
//    LinuxActionScript::ScriptStart("script.asc");
//    while(LinuxActionScript::m_is_running == 1) sleep(10);

//}
