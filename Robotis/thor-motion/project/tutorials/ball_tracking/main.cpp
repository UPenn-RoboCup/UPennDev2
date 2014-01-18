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

#include "sensor/vision/camera.h"
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



int main(void)
{
    printf( "\n===== Head tracking Tutorial for Thor =====\n\n");

    minIni* ini = new minIni(INI_FILE_PATH);
    Image* rgb_ball = new Image(Camera::WIDTH, Camera::HEIGHT, Image::RGB_PIXEL_SIZE);

    Camera::GetInstance()->Initialize(0);
    Camera::GetInstance()->LoadINISettings(ini);

    mjpg_streamer* streamer = new mjpg_streamer(Camera::WIDTH, Camera::HEIGHT);

    ColorFinder* ball_finder = new ColorFinder();
    ball_finder->LoadINISettings(ini);
    httpd::ball_finder = ball_finder;

    BallTracker tracker = BallTracker();

	//////////////////// Framework Initialize ////////////////////////////
	if(MotionManager::GetInstance()->Initialize() == false)
	{
		printf("Fail to initialize Motion Manager!\n");
		return 0;
	}

	MotionManager::GetInstance()->AddModule((MotionModule*)Head::GetInstance());
	MotionManager::GetInstance()->StartTimer();


	MotionStatus::m_EnableList[28].uID = "Head";
	MotionStatus::m_EnableList[29].uID = "Head";
	/////////////////////////////////////////////////////////////////////
	Head::GetInstance()->SetPanDXLPgain(8);
	Head::GetInstance()->SetTiltDXLPgain(8);
	Head::GetInstance()->MoveToHome();
	printf( "\n===== Start =====\n\n");
	while(1)
	{
		if(kbhit())
			if(_getch()==27)
				break;

		Camera::GetInstance()->CaptureFrame();

        tracker.Process(ball_finder->GetPosition(Camera::GetInstance()->fbuffer->m_HSVFrame));

		rgb_ball = Camera::GetInstance()->fbuffer->m_RGBFrame;
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
        usleep(1);
    }

    return 0;
}

