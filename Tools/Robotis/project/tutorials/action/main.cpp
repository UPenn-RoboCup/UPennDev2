/*
 * main.cpp
 *
 *  Created on: 2013. 1. 3.
 *      Author: zerom
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <termios.h>
#include <term.h>
#include <fcntl.h>
#include <ncurses.h>
#include <libgen.h>

#include "Thor.h"


using namespace Thor;

#define MOTION_FILE_PATH  "motion_4096.bin"
#define INI_FILE_PATH       "config.ini"

void change_current_dir()
{
    char exepath[1024] = {0};
    if(readlink("/proc/self/exe", exepath, sizeof(exepath)) != -1)
        chdir(dirname(exepath));
}

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
    printf( "\n===== Action script Tutorial for Thor =====\n\n");

    change_current_dir();

    Action::GetInstance()->LoadFile(MOTION_FILE_PATH);

    MotionManager *manager = MotionManager::GetInstance();

    if(manager->Initialize() == false)
    {
    	printf("Fail to initialize MotionManager.\n");
    	return 1;
    }

//    if(manager->InitforTest() == false)
//    {
//    	printf("Fail to initialize vel & accel value.\n");
//    	return 1;
//    }

    minIni* ini = new minIni(INI_FILE_PATH);

    manager->LoadINISettings(ini);
    manager->AddModule((MotionModule*)Action::GetInstance());

    printf("Press the ENTER key to begin!\n");
    getchar();

    manager->StartTimer();
    Action::GetInstance()->Start(1);
    int iPageNum = 1;
    int iRepeatCount = 1;
    while(true)
    {
    	if(kbhit())
    	{
    		char cmd = _getch();
    		if(cmd == 27)
    		{
    			Action::GetInstance()->Stop();
        		while(Action::GetInstance()->IsRunning())
        			sleep(8);
    			break;
    		}
    		else if(cmd == 'a')
    		{
    			printf("a \n");
    			manager->comm2 = true;
    			usleep(16000);
    			int result = manager->WriteWord(19, 592, 0, 0);
    			manager->comm2 = false;
    			if(result != COMM_RXSUCCESS)
    				printf("fail to write\n");
    			else
    				printf("success to write\n");
    		}
    		else if(cmd == 'b')
    		{
    			printf("b \n");
    			manager->comm2 = true;
    			usleep(16000);
    			int result = manager->WriteWord(19, 592, 1, 0);
    			manager->comm2 = false;
    			if(result != COMM_RXSUCCESS)
    				printf("fail to write\n");
    			else
    				printf("success to write\n");
    		}
    		else if(cmd == 'c')
    		{
    			printf("c \n");
    			manager->comm2 = true;
    			usleep(16000);
    			int result = manager->WriteWord(19, 592, 2, 0);
    			manager->comm2 = false;
    			if(result != COMM_RXSUCCESS)
    				printf("fail to write\n");
    			else
    				printf("success to write\n");

    		}
    		else
    			continue;
    	}


    	if(!Action::GetInstance()->IsRunning())
    	{
    		char cmd;

    		printf("Re? (y/n)");
    		cmd = _getch();
    		if(cmd == 'y')
    		{
    			iRepeatCount += 1;
    			printf("y , RepeatCount = %d\n", iRepeatCount);

    			Action::GetInstance()->Start(1);
    		}
    		else if(cmd == 27)
    		{
    			printf("exit \n");
    			break;
    		}
    		else if(cmd == 'n')
    		{
    			printf("n \n");
    			break;
    		}
    		else if(cmd == 'a')
    		{
    			printf("a \n");
    			manager->comm2 = true;
    			usleep(16000);
    			int result = manager->WriteWord(19, 592, 0, 0);
    			manager->comm2 = false;
    			if(result != COMM_RXSUCCESS)
    				printf("fail to write\n");
    			else
    				printf("success to write\n");
    		}
    		else if(cmd == 'b')
    		{
    			printf("b \n");
    			manager->comm2 = true;
    			usleep(16000);
    			int result = manager->WriteWord(19, 592, 1, 0);
    			manager->comm2 = false;
    			if(result != COMM_RXSUCCESS)
    				printf("fail to write\n");
    			else
    				printf("success to write\n");
    		}
    		else if(cmd == 'c')
    		{
    			printf("c \n");
    			manager->comm2 = true;
    			usleep(16000);
    			for(unsigned int index = 0; index < MotionStatus::m_CurrentJoints.size(); index++)
    			{
    				int id = MotionStatus::m_CurrentJoints[index].m_ID;
    				MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteByte(id, PRO54::P_TORQUE_ENABLE, 0, 0);
    			}
    			int result = manager->WriteWord(19, 592, 2, 0);
    			manager->comm2 = false;
    			if(result != COMM_RXSUCCESS)
    				printf("fail to write\n");
    			else
    				printf("success to write\n");
    		}
    	}

    	//sleep(1);
    }

    manager->StopTimer();

    return 0;
}
