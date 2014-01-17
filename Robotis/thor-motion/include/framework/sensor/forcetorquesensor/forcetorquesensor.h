/*
 * forcetorquesensor.h
 *
 *  Created on: 2013. 5. 25.
 *      Author: hjsong
 */

#ifndef _FORCETORQUESENSOR_H_
#define _FORCETORQUESENSOR_H_

#include <stdio.h>
#include "framework/minIni.h"

#define FTSENSOR_SECTION "FT Sensor"

namespace Thor
{
	class FTSensor
	{
	private:
		double daCalibrationMatrix[6][6];
		double dCalibrationMatrixGain;
		double daUnlodedVoltageOutput[6];
		double daCurrentVoltageOutput[6];
		double daDiffVoltage[6];
		double Fx, Fy, Fz, Tx, Ty, Tz;

	public:
		FTSensor();
		~FTSensor();

		void setCalibrationMatrix(double c00, double c01, double c02, double c03, double c04, double c05,
								  double c10, double c11, double c12, double c13, double c14, double c15,
								  double c20, double c21, double c22, double c23, double c24, double c25,
								  double c30, double c31, double c32, double c33, double c34, double c35,
								  double c40, double c41, double c42, double c43, double c44, double c45,
								  double c50, double c51, double c52, double c53, double c54, double c55 );
		void setCalibrationMatrixGain(double gain);
		void setUnlodedVoltageOutput(double g0, double g1, double g2, double g3, double g4, double g5);
		void setCurrentVoltageOutPut(double g0, double g1, double g2, double g3, double g4, double g5);
		void getForceTorque(double* fx, double* fy, double* fz, double* tx, double* ty, double* tz);

		void LoadINISettings(minIni* ini);
		void LoadINISettings(minIni* ini, const std::string &section);

		void CheckSensorSetting();

	};
}

#endif
