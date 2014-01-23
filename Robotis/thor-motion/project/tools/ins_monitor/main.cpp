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
#include "framework/Thor.h"

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
    printf( "\n===== INS monitor Tutorial for Thor =====\n\n");

    Ins* ins = new Ins();




    if(ins->Connect("ttyACM0", 921600) !=MIP_INTERFACE_OK)
    {
    	printf("fail to connect\n");
    	return 0;
    }

    //MotionManager::GetInstance()->Initialize();


    printf( "\n===== start initializing ins  =====\n\n");
    if(ins->Initialize() != MIP_INTERFACE_OK)
    {
    	printf("fail to init\n");
    	return 0;
    }

    printf( "\n===== set enalble data callback =====\n\n");
	//ins->SetEnableNavDataCallBack();
	ins->SetEnableAHRSDataCallBack();
	//ins->SetEnableGPSDataCallBack();

    printf("Press the ENTER key to begin!\n");
    getchar();

    ins->StartSensingDataUpdate();

    while(ins->IsRunning())
    {
    	if(kbhit())
    		if(_getch() == 27)
    		{
    			ins->StopSensingDataUpdate();
    			printf("\n");
    			break;
    		}

//    	printf("FB_GYRO : %d, RL_GYRO : %d, FB_ACCEL : %d, RL_ACCEL: %d\n",
//    			ins->strGyroDataforThor.y, ins->strGyroDataforThor.x, ins->strAccelDataforThor.y, ins->strAccelDataforThor.x);
//    	printf("FB_GYRO : %.3f, RL_GYRO : %.3f, FB_ACCEL : %.3f, RL_ACCEL: %.3f, EulerAngleX: %.3f,  EulerAngleY: %.3f,   EulerAngleZ: %.3f",
//    			MotionStatus::FB_GYRO, MotionStatus::RL_GYRO, MotionStatus::FB_ACCEL, MotionStatus::RL_ACCEL, MotionStatus::EulerAngleX, MotionStatus::EulerAngleY, MotionStatus::EulerAngleZ);
    	printf("FB_GYRO : %.3f \t RL_GYRO : %.3f \t EulerAngleX: %.3f \t EulerAngleY: %3.3f",
    			MotionStatus::FB_GYRO, MotionStatus::RL_GYRO, MotionStatus::EulerAngleX, MotionStatus::EulerAngleY);
//    	printf("FB_GYRO : %f, RL_GYRO : %f, FB_ACCEL : %f, RL_ACCEL: %f",
//    			MotionStatus::FB_GYRO, MotionStatus::RL_GYRO, MotionStatus::FB_ACCEL, MotionStatus::RL_ACCEL);

    	printf("\r");
//    	printf("%f   %f    %f\n",
//    			ins->curr_thor_gyro.scaled_gyro[0], ins->curr_thor_gyro.scaled_gyro[1], ins->curr_thor_gyro.scaled_gyro[2]);
    	usleep(8000);
    }

    ins->Disconnect();
    return 0;
}
