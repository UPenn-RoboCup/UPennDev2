/*
 * forcetorquesensor.cpp
 *
 *  Created on: 2013. 5. 25.
 *      Author: hjsong
 */



#include "framework/sensor/forcetorquesensor/forcetorquesensor.h"

using namespace Thor;

static bool isSetCalibrationMatrix = false;
static bool isSetUnlodedVoltageOutput = false;
static bool isSetCalibrationMatrixGain = false;
static const double invalid = 0.0;

FTSensor::FTSensor()
{
	Fx = Fy = Fz = Tx = Ty = Tz = 0.0;
	dCalibrationMatrixGain = 0.0;


	unsigned int i, j;

	for(i = 0;  i < 6 ; i++)
	{
		daCurrentVoltageOutput[i] = 0.0;
		daUnlodedVoltageOutput[i] = 0.0;
		daDiffVoltage[i] = 0.0;

		for(j = 0 ; j < 6; j++)
		{
			daCalibrationMatrix[i][j] = 0.0;
		}
	}
}

FTSensor::~FTSensor()
{


}

void FTSensor::setCalibrationMatrix(double c00, double c01, double c02, double c03, double c04, double c05,
									double c10, double c11, double c12, double c13, double c14, double c15,
									double c20, double c21, double c22, double c23, double c24, double c25,
									double c30, double c31, double c32, double c33, double c34, double c35,
									double c40, double c41, double c42, double c43, double c44, double c45,
									double c50, double c51, double c52, double c53, double c54, double c55 )
{
	daCalibrationMatrix[0][0] = c00;  daCalibrationMatrix[0][1] = c01;  daCalibrationMatrix[0][2] = c02;  daCalibrationMatrix[0][3] = c03;  daCalibrationMatrix[0][4] = c04;  daCalibrationMatrix[0][5] = c05;
	daCalibrationMatrix[1][0] = c10;  daCalibrationMatrix[1][1] = c11;  daCalibrationMatrix[1][2] = c12;  daCalibrationMatrix[1][3] = c13;  daCalibrationMatrix[1][4] = c14;  daCalibrationMatrix[1][5] = c15;
	daCalibrationMatrix[2][0] = c20;  daCalibrationMatrix[2][1] = c21;  daCalibrationMatrix[2][2] = c22;  daCalibrationMatrix[2][3] = c23;  daCalibrationMatrix[2][4] = c24;  daCalibrationMatrix[2][5] = c25;
	daCalibrationMatrix[3][0] = c30;  daCalibrationMatrix[3][1] = c31;  daCalibrationMatrix[3][2] = c32;  daCalibrationMatrix[3][3] = c33;  daCalibrationMatrix[3][4] = c34;  daCalibrationMatrix[3][5] = c35;
	daCalibrationMatrix[4][0] = c40;  daCalibrationMatrix[4][1] = c41;  daCalibrationMatrix[4][2] = c42;  daCalibrationMatrix[4][3] = c43;  daCalibrationMatrix[4][4] = c44;  daCalibrationMatrix[4][5] = c45;
	daCalibrationMatrix[5][0] = c50;  daCalibrationMatrix[5][1] = c51;  daCalibrationMatrix[5][2] = c52;  daCalibrationMatrix[5][3] = c53;  daCalibrationMatrix[5][4] = c54;  daCalibrationMatrix[5][5] = c55;

	for(unsigned int i = 0;  i < 6 ; i++)
	{
		for(unsigned int j = 0 ; j < 6; j++)
		{
			if(	daCalibrationMatrix[i][j] == 0.0)
			{
				fprintf(stderr, "Please set the correct CalibrationMatrix\n");
				isSetCalibrationMatrixGain = false;

				for(unsigned int row = 0; row < 6; row++)
				{
					for(unsigned int column = 0; column < 6 ; column++)
					{
						daCalibrationMatrix[i][j] = 0.0;
					}
				}
				return;
			}
		}
	}

	isSetCalibrationMatrix = true;
}


void FTSensor::setCalibrationMatrixGain(double gain)
{
	if(gain < 0.5 || gain > 2.5)
	{
		fprintf(stderr, "Please set the correct CalibrationMatrixGain\n");
		isSetCalibrationMatrixGain = false;
		return;
	}

	dCalibrationMatrixGain = gain;
	isSetCalibrationMatrixGain = true;
}


void FTSensor::setUnlodedVoltageOutput(double g0, double g1, double g2, double g3, double g4, double g5)
{

	if(g0 < 0.8 || g0 > 2.4)
	{
		fprintf(stderr, "Please set the correct unloded voltage\n");
		isSetUnlodedVoltageOutput = false;
		return;
	}
	if(g1 < 0.8 || g1 > 2.4)
	{
		fprintf(stderr, "Please set the correct unloded voltage\n");
		isSetUnlodedVoltageOutput = false;
		return;
	}
	if(g2 < 0.8 || g2 > 2.4)
	{
		fprintf(stderr, "Please set the correct unloded voltage\n");
		isSetUnlodedVoltageOutput = false;
		return;
	}
	if(g3 < 0.8 || g3 > 2.4)
	{
		fprintf(stderr, "Please set the correct unloded voltage\n");
		isSetUnlodedVoltageOutput = false;
		return;
	}
	if(g4 < 0.8 || g4 > 2.4)
	{
		fprintf(stderr, "Please set the correct unloded voltage\n");
		isSetUnlodedVoltageOutput = false;
		return;
	}
	if(g5 < 0.8 || g5 > 2.4)
	{
		fprintf(stderr, "Please set the correct unloded voltage\n");
		isSetUnlodedVoltageOutput = false;
		return;
	}

	daUnlodedVoltageOutput[0] = g0;
	daUnlodedVoltageOutput[1] = g1;
	daUnlodedVoltageOutput[2] = g2;
	daUnlodedVoltageOutput[3] = g3;
	daUnlodedVoltageOutput[4] = g4;
	daUnlodedVoltageOutput[5] = g5;

	isSetUnlodedVoltageOutput = true;
}

void FTSensor::setCurrentVoltageOutPut(double g0, double g1, double g2, double g3, double g4, double g5)
{
	if(!isSetUnlodedVoltageOutput)
	{
		fprintf(stderr, "Please set the correct unloded voltage\n");
		return;
	}

	daCurrentVoltageOutput[0] = g0;
	daCurrentVoltageOutput[1] = g1;
	daCurrentVoltageOutput[2] = g2;
	daCurrentVoltageOutput[3] = g3;
	daCurrentVoltageOutput[4] = g4;
	daCurrentVoltageOutput[5] = g5;

	daDiffVoltage[0] = daCurrentVoltageOutput[0] - daUnlodedVoltageOutput[0];
	daDiffVoltage[1] = daCurrentVoltageOutput[1] - daUnlodedVoltageOutput[1];
	daDiffVoltage[2] = daCurrentVoltageOutput[2] - daUnlodedVoltageOutput[2];
	daDiffVoltage[3] = daCurrentVoltageOutput[3] - daUnlodedVoltageOutput[3];
	daDiffVoltage[4] = daCurrentVoltageOutput[4] - daUnlodedVoltageOutput[4];
	daDiffVoltage[5] = daCurrentVoltageOutput[5] - daUnlodedVoltageOutput[5];
}

void FTSensor::getForceTorque(double* fx, double* fy, double* fz, double* tx, double* ty, double* tz)
{
	unsigned int i , j;

	if(!isSetCalibrationMatrixGain)
	{
		fprintf(stderr, "Please set the correct CalibrationMatrixGain\n");
		return;
	}

	if(!isSetUnlodedVoltageOutput)
	{
		fprintf(stderr, "Please Set the Correct Unloded Voltage\n");
		return;
	}

	if(!isSetCalibrationMatrix)
	{
		fprintf(stderr, "Please Set the Correct CalibrationMatrix\n");
		return;
	}

	double result[6] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

	for(i = 0; i < 6; i++)
	{
		for(j = 0; j < 6; j++)
		{
			result[i] += daCalibrationMatrix[i][j]*daDiffVoltage[j]*dCalibrationMatrixGain;
		}
	}

	Fx = result[0];
	Fy = result[1];
	Fz = result[2];
	Tx = result[3];
	Ty = result[4];
	Tz = result[5];

	*fx = Fx;
	*fy = Fy;
	*fz = Fz;
	*tx = Tx;
	*ty = Ty;
	*tz = Tz;
}


void FTSensor::LoadINISettings(minIni* ini)
{

	LoadINISettings(ini, FTSENSOR_SECTION);

}


void FTSensor::LoadINISettings(minIni* ini, const std::string &section)
{
	double value = invalid;

    if((value = ini->getd(section, "c00", invalid)) != invalid)                daCalibrationMatrix[0][0] = value;
    if((value = ini->getd(section, "c01", invalid)) != invalid)                daCalibrationMatrix[0][1] = value;
    if((value = ini->getd(section, "c02", invalid)) != invalid)                daCalibrationMatrix[0][2] = value;
    if((value = ini->getd(section, "c03", invalid)) != invalid)                daCalibrationMatrix[0][3] = value;
    if((value = ini->getd(section, "c04", invalid)) != invalid)                daCalibrationMatrix[0][4] = value;
    if((value = ini->getd(section, "c05", invalid)) != invalid)                daCalibrationMatrix[0][5] = value;

    if((value = ini->getd(section, "c10", invalid)) != invalid)                daCalibrationMatrix[1][0] = value;
    if((value = ini->getd(section, "c11", invalid)) != invalid)                daCalibrationMatrix[1][1] = value;
    if((value = ini->getd(section, "c12", invalid)) != invalid)                daCalibrationMatrix[1][2] = value;
    if((value = ini->getd(section, "c13", invalid)) != invalid)                daCalibrationMatrix[1][3] = value;
    if((value = ini->getd(section, "c14", invalid)) != invalid)                daCalibrationMatrix[1][4] = value;
    if((value = ini->getd(section, "c15", invalid)) != invalid)                daCalibrationMatrix[1][5] = value;

    if((value = ini->getd(section, "c20", invalid)) != invalid)                daCalibrationMatrix[2][0] = value;
    if((value = ini->getd(section, "c21", invalid)) != invalid)                daCalibrationMatrix[2][1] = value;
    if((value = ini->getd(section, "c22", invalid)) != invalid)                daCalibrationMatrix[2][2] = value;
    if((value = ini->getd(section, "c23", invalid)) != invalid)                daCalibrationMatrix[2][3] = value;
    if((value = ini->getd(section, "c24", invalid)) != invalid)                daCalibrationMatrix[2][4] = value;
    if((value = ini->getd(section, "c25", invalid)) != invalid)                daCalibrationMatrix[2][5] = value;

    if((value = ini->getd(section, "c30", invalid)) != invalid)                daCalibrationMatrix[3][0] = value;
    if((value = ini->getd(section, "c31", invalid)) != invalid)                daCalibrationMatrix[3][1] = value;
    if((value = ini->getd(section, "c32", invalid)) != invalid)                daCalibrationMatrix[3][2] = value;
    if((value = ini->getd(section, "c33", invalid)) != invalid)                daCalibrationMatrix[3][3] = value;
    if((value = ini->getd(section, "c34", invalid)) != invalid)                daCalibrationMatrix[3][4] = value;
    if((value = ini->getd(section, "c35", invalid)) != invalid)                daCalibrationMatrix[3][5] = value;

    if((value = ini->getd(section, "c40", invalid)) != invalid)                daCalibrationMatrix[4][0] = value;
    if((value = ini->getd(section, "c41", invalid)) != invalid)                daCalibrationMatrix[4][1] = value;
    if((value = ini->getd(section, "c42", invalid)) != invalid)                daCalibrationMatrix[4][2] = value;
    if((value = ini->getd(section, "c43", invalid)) != invalid)                daCalibrationMatrix[4][3] = value;
    if((value = ini->getd(section, "c44", invalid)) != invalid)                daCalibrationMatrix[4][4] = value;
    if((value = ini->getd(section, "c45", invalid)) != invalid)                daCalibrationMatrix[4][5] = value;

    if((value = ini->getd(section, "c50", invalid)) != invalid)                daCalibrationMatrix[5][0] = value;
    if((value = ini->getd(section, "c51", invalid)) != invalid)                daCalibrationMatrix[5][1] = value;
    if((value = ini->getd(section, "c52", invalid)) != invalid)                daCalibrationMatrix[5][2] = value;
    if((value = ini->getd(section, "c53", invalid)) != invalid)                daCalibrationMatrix[5][3] = value;
    if((value = ini->getd(section, "c54", invalid)) != invalid)                daCalibrationMatrix[5][4] = value;
    if((value = ini->getd(section, "c55", invalid)) != invalid)                daCalibrationMatrix[5][5] = value;

    if((value = ini->getd(section, "CalibrationMatrixGain", invalid)) != invalid)                dCalibrationMatrixGain = value;

    if((value = ini->getd(section, "UnlodedVoltageOutput0", invalid)) != invalid)                daUnlodedVoltageOutput[0] = value;
    if((value = ini->getd(section, "UnlodedVoltageOutput1", invalid)) != invalid)                daUnlodedVoltageOutput[1] = value;
    if((value = ini->getd(section, "UnlodedVoltageOutput2", invalid)) != invalid)                daUnlodedVoltageOutput[2] = value;
    if((value = ini->getd(section, "UnlodedVoltageOutput3", invalid)) != invalid)                daUnlodedVoltageOutput[3] = value;
    if((value = ini->getd(section, "UnlodedVoltageOutput4", invalid)) != invalid)                daUnlodedVoltageOutput[4] = value;
    if((value = ini->getd(section, "UnlodedVoltageOutput5", invalid)) != invalid)                daUnlodedVoltageOutput[5] = value;

    isSetCalibrationMatrix = isSetCalibrationMatrixGain = isSetUnlodedVoltageOutput = true;
 }


void FTSensor::CheckSensorSetting()
{
	fprintf(stderr, "Calibration Matrix\n");
	for(int i = 0 ; i < 6; i++)
	{
		fprintf(stderr, "\n");
		for(int j= 0 ; j <6; j++)
		{
			fprintf(stderr, "%f\t", daCalibrationMatrix[i][j]);
		}
	}
	fprintf(stderr, "\n");

	fprintf(stderr, "Calibration Matrix Gain : %f", dCalibrationMatrixGain);

	fprintf(stderr, "\n");
	for(int i = 0; i < 6 ; i++)
	{
		fprintf(stderr, "Unloded Voltage Output : %f\n", daUnlodedVoltageOutput[i]);
	}

	fprintf(stderr, "\n");
	fprintf(stderr, "Sensor Setting Check Complete!\n");
}
