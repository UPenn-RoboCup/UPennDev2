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
#include "framework/dynamixel.h"

using namespace Thor;

Dynamixel DXL0("/dev/ttyUSB2");
Dynamixel DXL2("/dev/ttyUSB3");

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
void Scan()
{
	fprintf(stderr, "\n");
	PingInfo *data = new PingInfo();
	for(int i = 1; i < 253; i++)
	{
		if(DXL0.Ping(i, data, 0) == COMM_RXSUCCESS)
		{
			fprintf(stderr, "\n                                          ... SUCCESS \r");
			fprintf(stderr, " [ID:%.3d] Model No : %.5d (0x%.2X 0x%.2X) \n",
					data->ID, data->ModelNumber, DXL_LOBYTE(data->ModelNumber), DXL_HIBYTE(data->ModelNumber));
		}
		else
			fprintf(stderr, ".");

		if(kbhit())
		{
			char c = _getch();
			if(c == 0x1b)
				break;
		}
	}
	fprintf(stderr, "\n\n");

	for(int i = 1; i < 253; i++)
	{
		if(DXL2.Ping(i, data, 0) == COMM_RXSUCCESS)
		{
			fprintf(stderr, "\n                                          ... SUCCESS \r");
			fprintf(stderr, " [ID:%.3d] Model No : %.5d (0x%.2X 0x%.2X) \n",
					data->ID, data->ModelNumber, DXL_LOBYTE(data->ModelNumber), DXL_HIBYTE(data->ModelNumber));
		}
		else
			fprintf(stderr, ".");

		if(kbhit())
		{
			char c = _getch();
			if(c == 0x1b)
				break;
		}
	}
	fprintf(stderr, "\n\n");
}

void FSRTest()
{
	int result;
	int value1 , value2, value3, value4;

	while(true)
	{
		if(kbhit())
			break;

//		result = DXL0.ReadWord(21, 626, &value1, 0);
//		result = DXL0.ReadWord(21, 628, &value2, 0);
//		result = DXL0.ReadWord(21, 630, &value3, 0);
//		result = DXL0.ReadWord(21, 632, &value4, 0);
//		fprintf(stderr, " READ VALUE1 : %d , %d , %d , %d    ", value1, value2, value3, value4);
//		fprintf(stderr, "\r");

		result = DXL2.ReadWord(22, 626, &value1, 0);
		result = DXL2.ReadWord(22, 628, &value2, 0);
		result = DXL2.ReadWord(22, 630, &value3, 0);
		result = DXL2.ReadWord(22, 632, &value4, 0);
		fprintf(stderr, " READ VALUE2 : %d , %d , %d , %d ", value1, value2, value3, value4);
		fprintf(stderr, "\r");
	}
}

void RFTTest()
{
	int result;
	int value1 , value2, value3, value4, value5, value6;

//	result = DXL2.ReadWord(23, 626, &value1, 0);
//	result = DXL2.ReadWord(23, 628, &value2, 0);
//	result = DXL2.ReadWord(23, 630, &value3, 0);
//	result = DXL2.ReadWord(23, 632, &value4, 0);
//	result = DXL2.ReadWord(21, 626, &value5, 0);
//	result = DXL2.ReadWord(21, 628, &value6, 0);
//
//	fprintf(stderr, "first read value\n", value1, value2, value3, value4, value5, value6);
//	fprintf(stderr, "READ VALUE : %d   %d    %d    %d  %d  %d\n", value1, value2, value3, value4, value5, value6);

	while(true)
	{
		if(kbhit())
			break;

		result = DXL0.ReadWord(25, 626, &value1, 0);
		result = DXL0.ReadWord(25, 628, &value2, 0);
		result = DXL0.ReadWord(25, 630, &value3, 0);
		result = DXL0.ReadWord(25, 632, &value4, 0);
		result = DXL0.ReadWord(23, 626, &value5, 0);
		result = DXL0.ReadWord(23, 628, &value6, 0);
		fprintf(stderr, "READ VALUE1 : %d   %d    %d    %d  %d  %d     ", value1, value2, value3, value4, value5, value6);
		fprintf(stderr, "\r");
	}
	fprintf(stderr, "\n");
}

void LFTTest()
{
	int result;
	int value1 , value2, value3, value4, value5, value6;
//
//	result = DXL0.ReadWord(24, 626, &value1, 0);
//	result = DXL0.ReadWord(24, 628, &value2, 0);
//	result = DXL0.ReadWord(24, 630, &value3, 0);
//	result = DXL0.ReadWord(24, 632, &value4, 0);
//	result = DXL0.ReadWord(22, 626, &value5, 0);
//	result = DXL0.ReadWord(22, 628, &value6, 0);
//
//	fprintf(stderr, "first read value\n", value1, value2, value3, value4, value5, value6);
//	fprintf(stderr, "READ VALUE : %d   %d    %d    %d  %d  %d\n", value1, value2, value3, value4, value5, value6);

	while(true)
	{
		if(kbhit())
			break;

		result = DXL2.ReadWord(26, 626, &value1, 0);
		result = DXL2.ReadWord(26, 628, &value2, 0);
		result = DXL2.ReadWord(26, 630, &value3, 0);
		result = DXL2.ReadWord(26, 632, &value4, 0);
		result = DXL2.ReadWord(24, 626, &value5, 0);
		result = DXL2.ReadWord(24, 628, &value6, 0);

		fprintf(stderr, "READ VALUE2 : %d   %d    %d    %d  %d  %d    ", value1, value2, value3, value4, value5, value6);
		fprintf(stderr, "\r");
	}
	fprintf(stderr, "\n");
}

int main(int argc, char *argv[])
{
    fprintf(stderr, "\n***********************************************************************\n");
    fprintf(stderr,   "*                     DXL Protocol 2.0 Monitor                        *\n");
    fprintf(stderr,   "***********************************************************************\n\n");



    if(DXL0.Connect() == false)
    {
    	fprintf(stderr, " Fail to open USB2Dyanmixel! [%d] \n\n", 0);
    	return 0;
    }

    if(DXL2.Connect() == false)
    {
    	fprintf(stderr, " Fail to open USB2Dyanmixel! [%d] \n\n", 2);
    	return 0;
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


    	if(strcmp(cmd, "scan") == 0)
    	{
    		Scan();
    	}
    	else if(strcmp(cmd, "baud") == 0)
    	{
    		if(num_param != 1)
    		{
    			fprintf(stderr, " Invalid parameters! \n");
    			continue;
    		}

    		if(DXL0.SetBaudrate(atoi(param[0])) == false)
    			fprintf(stderr, " Failed to change baudrate! \n");
    		else
    			fprintf(stderr, " Success to change baudrate! [ BAUD NUM: %d ]\n", atoi(param[0]));

    		if(DXL2.SetBaudrate(atoi(param[0])) == false)
    			fprintf(stderr, " Failed to change baudrate! \n");
    		else
    			fprintf(stderr, " Success to change baudrate! [ BAUD NUM: %d ]\n", atoi(param[0]));
    	}
    	else if(strcmp(cmd, "fsrt") == 0)
    	{
    		FSRTest();
    	}
    	else if(strcmp(cmd, "rftt")==0)
    	{
    		RFTTest();
    	}
    	else if(strcmp(cmd, "lftt")==0)
    	{
    		LFTTest();
    	}
    	else if(strcmp(cmd, "exit") == 0)
    	{
    		DXL0.Disconnect();
    		DXL2.Disconnect();
    		return 0;
    	}
    }
}


