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
#include "framework/sensor/lidar/lidar.h"

using namespace Thor;

Dynamixel DXL1("/dev/ttyUSB3");
Dynamixel DXL2("/dev/ttyUSB0");
Dynamixel DXL3("/dev/ttyUSB1");
Dynamixel DXL4("/dev/ttyUSB2");

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

void Usage(char *progname)
{
    fprintf(stderr, "-----------------------------------------------------------------------\n");
    fprintf(stderr, "Usage: %s\n" \
                    " [-h | --help]........: display this help\n" \
                    " [-d | --device]......: port to open                     (/dev/ttyUSB0)\n" \
                    , progname);
    fprintf(stderr, "-----------------------------------------------------------------------\n");
}

void Help()
{
	fprintf(stderr, "\n");
    fprintf(stderr, "COMMAND: \n\n" \
                    " baud [BAUD_NUM]           : Baudrate change to [BAUD_NUM] \n" \
                    "                              0:2400, 1:57600, 2:115200, 3:1M, 4:2M, 5:3M \n" \
                    "                              6:4M, 7:4.5M, 8:10.5M \n" \
                    " scan                      : Output the current status of all DXLs \n" \
                    " ping [ID] [ID] ...        : Output the current status of [ID] \n" \
                    " bp                        : Broadcast Ping \n" \
                    " wrb|w [ID] [ADDR] [VALUE] : Write 1 byte [VALUE] to [ADDR] of [ID] \n" \
                    " wrw [ID] [ADDR] [VALUE]   : Write 2 byte [VALUE] to [ADDR] of [ID] \n" \
                    " wrd [ID] [ADDR] [VALUE]   : Write 4 byte [VALUE] to [ADDR] of [ID] \n" \
                    " rdb [ID] [ADDR]           : Read 1 byte value from [ADDR] of [ID] \n" \
                    " rdw [ID] [ADDR]           : Read 2 byte value from [ADDR] of [ID] \n" \
                    " rdd [ID] [ADDR]           : Read 4 byte value from [ADDR] of [ID] \n" \
                    " r [ID] [ADDR] [LENGTH]    : Dumps the control table of [ID] \n" \
                    "                              [LENGTH] bytes from [ADDR] \n" \
                    " mon [ID] [ADDR] b|w|d     : Refresh byte|word|dword from [ADDR] of [ID] \n" \
                    " reboot [ID]               : Reboot the dynamixel of [ID] \n" \
                    " reset [ID] [OPTION]       : Factory reset the dynamixel of [ID] \n" \
                    "                              OPTION: 255(All), 1(Except ID), 2(Except ID&Baud)\n" \
                    " exit                      : Exit this program \n" \
                    );
	fprintf(stderr, "\n");
}

// Print error bit of status packet
void PrintErrorCode(int ErrorCode)
{
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

void Scan()
{
	fprintf(stderr, "\n");
	PingInfo *data = new PingInfo();
	for(int i = 1; i < 253; i++)
	{
		if(DXL1.Ping(i, data, 0) == COMM_RXSUCCESS)
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

		result = DXL2.ReadWord(24, 626, &value1, 0);
		result = DXL2.ReadWord(24, 628, &value2, 0);
		result = DXL2.ReadWord(24, 630, &value3, 0);
		result = DXL2.ReadWord(24, 632, &value4, 0);
		fprintf(stderr, "\n READ VALUE : %d , %d , %d , %d \n", value1, value2, value3, value4);
	}
}

void FTTest()
{
	int result;
	int value1 , value2, value3, value4;

	while(true)
	{
		if(kbhit())
			break;

		result = DXL1.ReadWord(11, 626, &value1, 0);
		result = DXL1.ReadWord(11, 628, &value2, 0);
		result = DXL1.ReadWord(11, 630, &value3, 0);
		result = DXL1.ReadWord(11, 632, &value4, 0);
		fprintf(stderr, "\n READ VALUE : %d , %d , %d , %d \n", value1, value2, value3, value4);
	}
}

void SensorMonitoring()
{
	int result;
	int value1 , value2, value3, value4;
	int value;
	printf("\n");
	DXL3.WriteWord(35, 32, 500, 0);
	int i = 0;
	while(true)
	{
		usleep(1000);
		i += 1;
		if(kbhit())
			break;

		result = DXL2.ReadWord(24, 626, &value1, 0);
		result = DXL2.ReadWord(24, 628, &value2, 0);
		result = DXL2.ReadWord(24, 630, &value3, 0);
		result = DXL2.ReadWord(24, 632, &value4, 0);
		result = DXL1.ReadWord(27, 626, &value, 0);

		if(i == 1)
		{
			DXL3.WriteWord(35, 30, 2548, 0);
			i = -1;
		}
		else
		{
			DXL3.WriteWord(35, 30, 1548, 0);
			i = 0;
		}

		//value -= 2100;
		//value *= 10;
		fprintf(stderr, "FT : %d  %d  %d  %d  ,  FT : %d  LIDAR   %f   %f",
								value1, value2, value3, value4, value , MotionStatus::lidar_data[0][500], MotionStatus::lidar_data[1][500]);
		fprintf(stderr, "\r");
	}
}

int main(int argc, char *argv[])
{
    fprintf(stderr, "\n***********************************************************************\n");
    fprintf(stderr,   "*                           Sensor Monitor                            *\n");
    fprintf(stderr,   "***********************************************************************\n\n");


    if(DXL1.Connect() == false)
    {
    	fprintf(stderr, " Fail to open USB2Dyanmixel! [dxl1] \n\n");
    	return 0;
    }

    if(DXL2.Connect() == false)
     {
     	fprintf(stderr, " Fail to open USB2Dyanmixel! [dxl2] \n\n");
     	return 0;
     }


    Thor::Lidar lidar("192.168.0.10");

    lidar.open_sensor();
    lidar.set_scanning_deg(-90.0, 90.0);
    lidar.start_measurement(URG_DISTANCE, URG_SCAN_INFINITY);
    lidar.StartSensingDataUpdate();

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
    	else if(strcmp(cmd, "fsrt") == 0)
    	{
    		FSRTest();
    	}
    	else if(strcmp(cmd, "ftt")==0)
    	{
    		FTTest();
    	}
    	else if(strcmp(cmd, "mon") == 0)
    	{
		SensorMonitoring();
    	}
    	else if(strcmp(cmd, "exit") == 0)
    	{
    		DXL1.Disconnect();
    		DXL2.Disconnect();
    		return 0;
    	}
    }

    lidar.StopSensingDataUpdate();
    lidar.close_sensor();

}


