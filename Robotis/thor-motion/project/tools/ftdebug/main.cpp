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
#include "framework/sensor/forcetorquesensor/forcetorquesensor.h"

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

	FTSensor RightLegFTSensor;
	minIni* ini = new minIni("config.ini");

	RightLegFTSensor.LoadINISettings(ini, "Right Leg FT Sensor");

	double FX, FY, FZ, TX, TY, TZ;
	double ZMP_X, ZMP_Y;

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

		RightLegFTSensor.setCurrentVoltageOutPut(3.3*value1/4096.0, 3.3*value2/4096.0, 3.3*value3/4096.0,
				3.3*value4/4096.0, 3.3*value5/4096.0, 3.3*value6/4096.0);
		RightLegFTSensor.getForceTorque(&FX, &FY, &FZ, &TX, &TY, &TZ);
		ZMP_X = 1000.0*(TY + 0.118*FX)/FZ;
		ZMP_Y = 1000.0*(-TX + 0.118*FY)/FZ;
		fprintf(stderr, "FX: %.3f FY: %.3f FZ: %.3f TX: %.3f TY: %.3f TZ: %.3f  ZMP_X: %.3f ZMP_Y: %.3f", FX, FY, FZ, TX, TY, TZ, ZMP_X, ZMP_Y);
		fprintf(stderr, "\r");
	}
	fprintf(stderr, "\n");
}

void GetRZMP()
{
	int result;
	int value1 , value2, value3, value4, value5, value6;


	FTSensor RightLegFTSensor;
	minIni* ini = new minIni("config.ini");

	RightLegFTSensor.LoadINISettings(ini, "Right Leg FT Sensor");

	double FX, FY, FZ, TX, TY, TZ;
	double ZMP_X, ZMP_Y;

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

		RightLegFTSensor.setCurrentVoltageOutPut(3.3*value1/4096.0, 3.3*value2/4096.0, 3.3*value3/4096.0,
				3.3*value4/4096.0, 3.3*value5/4096.0, 3.3*value6/4096.0);
		RightLegFTSensor.getForceTorque(&FX, &FY, &FZ, &TX, &TY, &TZ);
		ZMP_X = 1000.0*(TY + 0.118*FX)/FZ;
		ZMP_Y = 1000.0*(-TX + 0.118*FY)/FZ;
		fprintf(stderr, "FX: %.3f FY: %.3f FZ: %.3f TX: %.3f TY: %.3f TZ: %.3f  ZMP_X: %.3f ZMP_Y: %.3f", FX, FY, FZ, TX, TY, TZ, ZMP_X, ZMP_Y);
		fprintf(stderr, "\r");
	}
	fprintf(stderr, "\n");
}

void LFTTest()
{
	int result;
	int value1 , value2, value3, value4, value5, value6;

	FTSensor LeftLegFTSensor;
	minIni* ini = new minIni("config.ini");

	LeftLegFTSensor.LoadINISettings(ini, "Left Leg FT Sensor");

	double FX, FY, FZ, TX, TY, TZ;
	double ZMP_X, ZMP_Y;

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



		LeftLegFTSensor.setCurrentVoltageOutPut(3.3*value1/4096.0, 3.3*value2/4096.0, 3.3*value3/4096.0,
				3.3*value4/4096.0, 3.3*value5/4096.0, 3.3*value6/4096.0);
		LeftLegFTSensor.getForceTorque(&FX, &FY, &FZ, &TX, &TY, &TZ);
		ZMP_X = 1000.0*(TY + 0.118*FX)/FZ;
		ZMP_Y = 1000.0*(-TX + 0.118*FY)/FZ;
		fprintf(stderr, "FX: %.3f FY: %.3f FZ: %.3f TX: %.3f TY: %.3f TZ: %.3f  ZMP_X: %.3f ZMP_Y: %.3f", FX, FY, FZ, TX, TY, TZ, ZMP_X, ZMP_Y);
		fprintf(stderr, "\r");

		//fprintf(stderr, "\r");
	}
	fprintf(stderr, "\n");
}

void GetLZMP()
{
	int result;
	int value1 , value2, value3, value4, value5, value6;

	FTSensor LeftLegFTSensor;
	minIni* ini = new minIni("config.ini");

	LeftLegFTSensor.LoadINISettings(ini, "Left Leg FT Sensor");

	double FX, FY, FZ, TX, TY, TZ;
	double ZMP_X, ZMP_Y;

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



		LeftLegFTSensor.setCurrentVoltageOutPut(3.3*value1/4096.0, 3.3*value2/4096.0, 3.3*value3/4096.0,
				3.3*value4/4096.0, 3.3*value5/4096.0, 3.3*value6/4096.0);
		LeftLegFTSensor.getForceTorque(&FX, &FY, &FZ, &TX, &TY, &TZ);
		ZMP_X = 1000.0*(TY + 0.118*FX)/FZ;
		ZMP_Y = 1000.0*(-TX + 0.118*FY)/FZ;
		fprintf(stderr, "FX: %.3f FY: %.3f FZ: %.3f TX: %.3f TY: %.3f TZ: %.3f  ZMP_X: %.3f ZMP_Y: %.3f", FX, FY, FZ, TX, TY, TZ, ZMP_X, ZMP_Y);
		fprintf(stderr, "\r");

		//fprintf(stderr, "\r");
	}
	fprintf(stderr, "\n");
}



void GetZMP()
{

	int result;
	int value1 , value2, value3, value4, value5, value6;

	FTSensor RightLegFTSensor;
	minIni* ini = new minIni("config.ini");

	RightLegFTSensor.LoadINISettings(ini, "Right Leg FT Sensor");

	double R_FX_N, R_FY_N, R_FZ_N, R_TX_Nm, R_TY_Nm, R_TZ_Nm;
	double R_ZMP_X_mm, R_ZMP_Y_mm;


	FTSensor LeftLegFTSensor;
	LeftLegFTSensor.LoadINISettings(ini, "Left Leg FT Sensor");

	double L_FX_N, L_FY_N, L_FZ_N, L_TX_Nm, L_TY_Nm, L_TZ_Nm;
	double L_ZMP_X_mm, L_ZMP_Y_mm;

	double ZMP_X_mm, ZMP_Y_mm;

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
		RightLegFTSensor.setCurrentVoltageOutPut(3.3*value1/4096.0, 3.3*value2/4096.0, 3.3*value3/4096.0,
				3.3*value4/4096.0, 3.3*value5/4096.0, 3.3*value6/4096.0);
		RightLegFTSensor.getForceTorque(&R_FX_N, &R_FY_N, &R_FZ_N, &R_TX_Nm, &R_TY_Nm, &R_TZ_Nm);
		R_ZMP_X_mm = -1000.0*(R_TY_Nm + 0.118*R_FX_N)/R_FZ_N;
		R_ZMP_Y_mm = 1000.0*(-R_TX_Nm + 0.118*R_FY_N)/R_FZ_N;


		result = DXL2.ReadWord(26, 626, &value1, 0);
		result = DXL2.ReadWord(26, 628, &value2, 0);
		result = DXL2.ReadWord(26, 630, &value3, 0);
		result = DXL2.ReadWord(26, 632, &value4, 0);
		result = DXL2.ReadWord(24, 626, &value5, 0);
		result = DXL2.ReadWord(24, 628, &value6, 0);

		LeftLegFTSensor.setCurrentVoltageOutPut(3.3*value1/4096.0, 3.3*value2/4096.0, 3.3*value3/4096.0,
				3.3*value4/4096.0, 3.3*value5/4096.0, 3.3*value6/4096.0);
		LeftLegFTSensor.getForceTorque(&L_FX_N, &L_FY_N, &L_FZ_N, &L_TX_Nm, &L_TY_Nm, &L_TZ_Nm);
		L_ZMP_X_mm = -1000.0*(-L_TY_Nm + 0.118*L_FX_N)/L_FZ_N;
		L_ZMP_Y_mm = 1000.0*(-L_TX_Nm + 0.118*(-L_FY_N))/L_FZ_N;


		ZMP_X_mm = ((L_ZMP_X_mm +  0.0)*R_FZ_N + (R_ZMP_X_mm -  0.0)*L_FZ_N )/(R_FZ_N + L_FZ_N);
		ZMP_Y_mm = ((L_ZMP_Y_mm + 90.0)*R_FZ_N + (R_ZMP_Y_mm - 90.0)*L_FZ_N )/(R_FZ_N + L_FZ_N);


		fprintf(stderr, "ZMP_X : %f \t ZMP_Y : %f \t", ZMP_X_mm, ZMP_Y_mm);

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
    	else if(strcmp(cmd, "zmp")==0)
    	{
    		GetZMP();
    	}
    	else if(strcmp(cmd, "exit") == 0)
    	{
    		DXL0.Disconnect();
    		DXL2.Disconnect();
    		return 0;
    	}
    }
}


