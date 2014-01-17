/*
 * StepDataDefine.h
 *
 *  Created on: 2013. 12. 7.
 *      Author: hjsong
 */

#ifndef STEPDATADEFINE_H_
#define STEPDATADEFINE_H_

enum
{
	InWalkingStarting = 0,
	InWalking = 1,
	InWalkingEnding = 2
};

enum
{
	LFootMove = 1,
	RFootMove = 2,
	NFootMove = 3
};

namespace Thor
{
	typedef struct
	{
		double x, y, z, roll, pitch, yaw;
	} Pose3D;

	typedef struct
	{
		int bMovingFoot;
		double dFootHeight, dZ_Swap_Amplitude, dShoulderSwingGain, dElbowSwingGain;
		double dWaistPichAngle, dWaistYawAngle;
		Pose3D stLeftFootPosition;
		Pose3D stRightFootPosition;
		Pose3D stBodyPosition;
	} StepPositionData;

	typedef struct
	{
		int bWalkingState;
		double dAbsStepTime, dDSPratio;
		double sigmoid_ratio_x, sigmoid_ratio_y, sigmoid_ratio_z, sigmoid_ratio_roll, sigmoid_ratio_pitch, sigmoid_ratio_yaw;
		double sigmoid_distortion_x, sigmoid_distortion_y, sigmoid_distortion_z, sigmoid_distortion_roll, sigmoid_distortion_pitch, sigmoid_distortion_yaw;
	} StepTimeData;

	typedef struct
	{
		StepPositionData PositionData;
		StepTimeData TimeData;
	} StepData;
}


#endif /* STEPDATADEFINE_H_ */
