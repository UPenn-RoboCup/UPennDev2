/*
 * main.cpp
 *
 *  Created on: 2011. 1. 4.
 *      Author: robotis
 */

#include <unistd.h>
#include <string.h>
#include <libgen.h>

#include "minIni.h"
#include "math/point.h"
#include "streamer/mjpg_streamer.h"
#include "Thor.h"

#define INI_FILE_PATH       "config.ini"

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
    printf( "\n===== Color filtering Tutorial for Thor =====\n\n");

    minIni* ini = new minIni(INI_FILE_PATH);
    Image* rgb_ball = new Image(Camera::WIDTH, Camera::HEIGHT, Image::RGB_PIXEL_SIZE);

    Camera::GetInstance()->Initialize(0);
    Camera::GetInstance()->LoadINISettings(ini);

    mjpg_streamer* streamer = new mjpg_streamer(Camera::WIDTH, Camera::HEIGHT);

    ColorFinder* finder = new ColorFinder();
    finder->LoadINISettings(ini);
    httpd::ball_finder = finder;

    while(true)
    {
    	if(kbhit())
    		if(_getch()==27)
    			break;

    	Point2D pos;
    	Camera::GetInstance()->CaptureFrame();

    	memcpy(rgb_ball->m_ImageData, Camera::GetInstance()->fbuffer->m_RGBFrame->m_ImageData, Camera::GetInstance()->fbuffer->m_RGBFrame->m_ImageSize);

        pos = finder->GetPosition(Camera::GetInstance()->fbuffer->m_HSVFrame);

        fprintf(stderr, "posx: %f, posy: %f \r", pos.X, pos.Y);

        for(int i = 0; i < rgb_ball->m_NumberOfPixels; i++)
        {
            if(finder->m_result->m_ImageData[i] == 1)
            {
                rgb_ball->m_ImageData[i*rgb_ball->m_PixelSize + 0] = 255;
                rgb_ball->m_ImageData[i*rgb_ball->m_PixelSize + 1] = 0;
                rgb_ball->m_ImageData[i*rgb_ball->m_PixelSize + 2] = 0;
            }
        }

        streamer->send_image(rgb_ball);
    }

    return 0;
}
