/*
 * DXLInfo.h
 *
 *  Created on: 2013. 1. 14.
 *      Author: hjsong
 */

#ifndef DXLINFO_H_
#define DXLINFO_H_

namespace Thor
{

class DXLInfo
{

public:
	int m_SyncWriteStartAddr;
	int m_SyncWriteLength;
	const int GOAL_POSITION_ADDR;
	const int PRESENT_POSITION_ADDR;
	const int POSITION_LENGTH;
	const int PGAIN_ADDR;
	const int PGAIN_LENGTH;
	const int MAX_VALUE;
	const int MIN_VALUE;
	const int MODEL_NUM;

	virtual int Angle2Value(double angle) = 0;
	virtual double Value2Angle(int value) = 0;
	virtual int Rad2Value(double radian) = 0;
	virtual double Value2Rad(int value) = 0;
	virtual int ValueScaleUp(int value) = 0;
	virtual int ValueScaleDown(int value) = 0;


	DXLInfo(int goal_position_addr, int present_position_addr, int position_length, int pgain_addr, int pgain_length, int max_value, int min_value, int model_num) :
	GOAL_POSITION_ADDR(goal_position_addr), PRESENT_POSITION_ADDR(present_position_addr), POSITION_LENGTH(position_length), PGAIN_ADDR(pgain_addr), PGAIN_LENGTH(pgain_length),
	MAX_VALUE(max_value), MIN_VALUE(min_value), MODEL_NUM(model_num)
	{
		m_SyncWriteLength = position_length;
		m_SyncWriteStartAddr = goal_position_addr;
	}

};

} /* namespace Thor */
#endif /* DXLINFO_H_ */
