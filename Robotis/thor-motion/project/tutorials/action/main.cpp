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

//void change_current_dir()
//{
//    char exepath[1024] = {0};
//    if(readlink("/proc/self/exe", exepath, sizeof(exepath)) != -1)
//        chdir(dirname(exepath));
//}

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
			" autoplay       : forward walking\n"
			" climbblock    : block climbing \n" \
			"release        : hand release \n"\
			" exit          : Exit this program \n" \
	);

	//			" c             : walking down the block \n" \
	//			" d             : ??? \n" \//
	fprintf(stderr, "\n");
}


int MotionPage[4] = { 11 , 27, 23, 28 };
int MotionPageIdx = 0;


int main(void)
{
    printf( "\n===== Action script Tutorial for Thor =====\n\n");

//    change_current_dir();

    Action::GetInstance()->LoadFile(MOTION_FILE_PATH);

    MotionManager *manager = MotionManager::GetInstance();

    if(manager->Initialize() == false)
    {
    	printf("Fail to initialize MotionManager.\n");
    	return 1;
    }


    minIni* ini = new minIni(INI_FILE_PATH);

    manager->LoadINISettings(ini);
    manager->AddModule((MotionModule*)Action::GetInstance());

    printf("Press the ENTER key to begin!\n");
    getchar();


	for(unsigned int jointIndex = 0; jointIndex < 35; jointIndex++)
	{
		int id = jointIndex;

		//not be fixed code

		if(id >= 1 && id <= 14)
			MotionStatus::m_EnableList[id-1].uID = "Action";
		else if(id == 29 || id == 30 || id == 27 || id == 28)
			MotionStatus::m_EnableList[id-1].uID = "Action";
	}


    manager->StartTimer();

    Action::GetInstance()->Start(19, MotionStatus::m_EnableList);

    while(true)
    {
		if(Action::GetInstance()->IsRunning() == false)
			break;
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


		if(strcmp(cmd, "autoplay") == 0)
		{
			printf("Auto Motion Play Start\n");
			printf("if you want to stop, press ESC key!!\n");
			printf("Start Motion Page 11!!\n");
			printf("Press any key to start!!\n");
			_getch();

			Action::GetInstance()->Start(11, MotionStatus::m_EnableList);
			while(true)
			{

				if(Action::GetInstance()->IsRunning() == false)
				{
					int i = 0;
					while(true)
					{
						i = i + 1;

						usleep(30000);
						if( i == 165)
						{
							MotionPageIdx += 1;

							if(MotionPageIdx == 4)
								MotionPageIdx = 0;

							printf("Start Motion Page %d!!\n" , MotionPage[MotionPageIdx]);
							Action::GetInstance()->Start(MotionPage[MotionPageIdx], MotionStatus::m_EnableList);
							break;
						}

						if(kbhit())
							if(_getch() == 27)
								return 0;
					}
				}


				if(kbhit())
					if(_getch() == 27)
						return 0;

			}

		}
		else if(strcmp(cmd, "exit") == 0)
			return 0;


	}


    manager->StopTimer();

    return 0;
}



//    Action::GetInstance()->Start(1);
//    int iPageNum = 1;
//    int iRepeatCount = 1;
//    while(true)
//    {
//    	if(kbhit())
//    	{
//    		char cmd = _getch();
//    		if(cmd == 27)
//    		{
//    			Action::GetInstance()->Stop();
//        		while(Action::GetInstance()->IsRunning())
//        			sleep(8);
//    			break;
//    		}
//    		else if(cmd == 'a')
//    		{
//    			printf("a \n");
//    			manager->comm2 = true;
//    			usleep(16000);
//    			int result = manager->WriteWord(19, 592, 0, 0);
//    			manager->comm2 = false;
//    			if(result != COMM_RXSUCCESS)
//    				printf("fail to write\n");
//    			else
//    				printf("success to write\n");
//    		}
//    		else if(cmd == 'b')
//    		{
//    			printf("b \n");
//    			manager->comm2 = true;
//    			usleep(16000);
//    			int result = manager->WriteWord(19, 592, 1, 0);
//    			manager->comm2 = false;
//    			if(result != COMM_RXSUCCESS)
//    				printf("fail to write\n");
//    			else
//    				printf("success to write\n");
//    		}
//    		else if(cmd == 'c')
//    		{
//    			printf("c \n");
//    			manager->comm2 = true;
//    			usleep(16000);
//    			int result = manager->WriteWord(19, 592, 2, 0);
//    			manager->comm2 = false;
//    			if(result != COMM_RXSUCCESS)
//    				printf("fail to write\n");
//    			else
//    				printf("success to write\n");
//
//    		}
//    		else
//    			continue;
//    	}
//
//
//    	if(!Action::GetInstance()->IsRunning())
//    	{
//    		char cmd;
//
//    		printf("Re? (y/n)");
//    		cmd = _getch();
//    		if(cmd == 'y')
//    		{
//    			iRepeatCount += 1;
//    			printf("y , RepeatCount = %d\n", iRepeatCount);
//
//    			Action::GetInstance()->Start(1);
//    		}
//    		else if(cmd == 27)
//    		{
//    			printf("exit \n");
//    			break;
//    		}
//    		else if(cmd == 'n')
//    		{
//    			printf("n \n");
//    			break;
//    		}
//    		else if(cmd == 'a')
//    		{
//    			printf("a \n");
//    			manager->comm2 = true;
//    			usleep(16000);
//    			int result = manager->WriteWord(19, 592, 0, 0);
//    			manager->comm2 = false;
//    			if(result != COMM_RXSUCCESS)
//    				printf("fail to write\n");
//    			else
//    				printf("success to write\n");
//    		}
//    		else if(cmd == 'b')
//    		{
//    			printf("b \n");
//    			manager->comm2 = true;
//    			usleep(16000);
//    			int result = manager->WriteWord(19, 592, 1, 0);
//    			manager->comm2 = false;
//    			if(result != COMM_RXSUCCESS)
//    				printf("fail to write\n");
//    			else
//    				printf("success to write\n");
//    		}
//    		else if(cmd == 'c')
//    		{
//    			printf("c \n");
//    			manager->comm2 = true;
//    			usleep(16000);
//    			for(unsigned int index = 0; index < MotionStatus::m_CurrentJoints.size(); index++)
//    			{
//    				int id = MotionStatus::m_CurrentJoints[index].m_ID;
//    				MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteByte(id, PRO54::P_TORQUE_ENABLE, 0, 0);
//    			}
//    			int result = manager->WriteWord(19, 592, 2, 0);
//    			manager->comm2 = false;
//    			if(result != COMM_RXSUCCESS)
//    				printf("fail to write\n");
//    			else
//    				printf("success to write\n");
//    		}
//    	}
//
//    	//sleep(1);
//    }

//    manager->StopTimer();
//
//    return 0;
//}
