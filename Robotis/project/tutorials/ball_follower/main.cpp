/*
 * main.cpp
 *
 *  Created on: 2013. 3. 12.
 *      Author: hjsong
 */


#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <libgen.h>
#include "streamer/mjpg_streamer.h"
#include "Thor.h"

#define INI_FILE_PATH       "config.ini"

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
	printf( "\n===== Ball following Tutorial for Thor =====\n\n");

	Image* rgb_ball = new Image(Camera::WIDTH, Camera::HEIGHT, Image::RGB_PIXEL_SIZE);

	minIni* ini = new minIni(INI_FILE_PATH);

	Camera::GetInstance()->Initialize(0);
	Camera::GetInstance()->LoadINISettings(ini);

	mjpg_streamer* streamer = new mjpg_streamer(Camera::WIDTH, Camera::HEIGHT);

	ColorFinder* ball_finder = new ColorFinder();
	ball_finder->LoadINISettings(ini);
	httpd::ball_finder = ball_finder;

	BallTracker tracker = BallTracker();
	BallFollower follower = BallFollower();
	follower.DEBUG_PRINT = true;

	//////////////////// Framework Initialize ////////////////////////////
	if(MotionManager::GetInstance()->Initialize() == false)
	{
		printf("Fail to initialize Motion Manager!\n");
		return 0;
	}
//	if(MotionManager::GetInstance()->InitforTest() == false)
//	{
//		printf("Fail to initialize Motion ManagerInitforTest!\n");
//		return 0;
//	}

	MotionManager::GetInstance()->LoadINISettings(ini);

	for(unsigned int index = 0; index < MotionStatus::m_CurrentJoints.size(); index++)
	{
		int id = MotionStatus::m_CurrentJoints[index].m_ID;

		int err, val;
		MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteDWord(id, PRO54::P_GOAL_ACCELATION_LL, 0, &err);
		printf("id : %d  ", id);
		PrintErrorCode(err);

		usleep(1000);
	}

//	for(unsigned int index = 0; index < MotionStatus::m_CurrentJoints.size(); index++)
//	{
//		usleep(1);
//		int id = MotionStatus::m_CurrentJoints[index].m_ID;
//
//		if(id > 14 && id <27)
//			MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteWord(id, PRO54::P_VELOCITY_I_GAIN_L, 0, 0);
//	}

	/////////////////////////////////////////////////////////////////////

	//////////////////////// INS Initialize /////////////////////////////
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
	//////////////////////////////////////////////////////////////////

	printf("Press the ENTER key to move init pose!\n");
	getchar();

	MotionManager::GetInstance()->AddModule((MotionModule*)Action::GetInstance());
	Action::GetInstance()->LoadFile("motion_4096.bin");
	MotionManager::GetInstance()->StartTimer();
	Action::GetInstance()->Start(19);
	while(Action::GetInstance()->IsRunning())
		usleep(8000);


	for(int i =0; i < 27; i ++)
		MotionStatus::m_EnableList[i].uID = "Walking";

	MotionStatus::m_EnableList[28].uID = "Head";
	MotionStatus::m_EnableList[29].uID = "Head";
	MotionManager::GetInstance()->StopTimer();
	printf("Press the ENTER key to begin!\n");

	MotionManager::GetInstance()->AddModule((MotionModule*)Head::GetInstance());
	MotionManager::GetInstance()->AddModule((MotionModule*)Walking::GetInstance());
	Walking::GetInstance()->LoadINISettings(ini);
	Walking::GetInstance()->BALANCE_ENABLE = true;
	Walking::GetInstance()->P_GAIN = 64;
	MotionManager::GetInstance()->StartTimer();

	getchar();
	printf("Start Demonstration!\n");
	while(1)
	{
		if(kbhit())
			if(_getch()==27)
				break;

		Camera::GetInstance()->CaptureFrame();

		memcpy(rgb_ball->m_ImageData, Camera::GetInstance()->fbuffer->m_RGBFrame->m_ImageData, Camera::GetInstance()->fbuffer->m_RGBFrame->m_ImageSize);

		tracker.Process(ball_finder->GetPosition(Camera::GetInstance()->fbuffer->m_HSVFrame));

		if(tracker.GetTrackerStatus() == true)
		{
			if(Action::GetInstance()->IsRunning() == true)
			{
				Action::GetInstance()->Brake();
				MotionStatus::m_EnableList[28].uID = "Head";
				MotionStatus::m_EnableList[29].uID = "Head";
			}
		}
		else
		{
			if(Action::GetInstance()->IsRunning() == false)
			{
				MotionStatus::m_EnableList[28].uID = "Action";
				MotionStatus::m_EnableList[29].uID = "Action";
				Action::GetInstance()->Start(28, MotionStatus::m_EnableList);
			}
		}

		follower.Process(tracker.ball_position);

		for(int i = 0; i < rgb_ball->m_NumberOfPixels; i++)
		{
			if(ball_finder->m_result->m_ImageData[i] == 1)
			{
				rgb_ball->m_ImageData[i*rgb_ball->m_PixelSize + 0] = 255;
				rgb_ball->m_ImageData[i*rgb_ball->m_PixelSize + 1] = 0;
				rgb_ball->m_ImageData[i*rgb_ball->m_PixelSize + 2] = 0;
			}
		}

		streamer->send_image(rgb_ball);
	}

	Walking::GetInstance()->Stop();
	while(Walking::GetInstance()->IsRunning())
		usleep(1000);


	Action::GetInstance()->Stop();
	while(Action::GetInstance()->IsRunning())
		usleep(1000);

	ins->StopSensingDataUpdate();
	MotionManager::GetInstance()->StopTimer();

	delete streamer;
	delete ins;

	return 0;

}

