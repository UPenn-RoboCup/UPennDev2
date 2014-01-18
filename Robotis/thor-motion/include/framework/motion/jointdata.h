/*
 * JointData.h
 *
 *  Created on: 2013. 1. 15.
 *      Author: hjsong
 */

#ifndef JOINTDATA_H_
#define JOINTDATA_H_

#include "dxl_info.h"
#include "dxl_comm.h"
#include "framework/bulkread.h"

namespace Thor
{

class JointData
{
public:
	int m_ID;
	DXLInfo *m_DXLInfo;
	DXLComm *m_DXL_Comm;
	BulkRead *m_BulkRead;
	double m_Angle;
	int m_Value;
	int m_Pgain;
	int m_Igain;
	int m_Dgain;

	JointData(int ID, DXLInfo *DXL_Info) : m_ID(ID),
			m_Angle(0.0), m_Pgain(32), m_Igain(0), m_Dgain(0)
	{
		m_DXLInfo = DXL_Info;
		if(DXL_Info->POSITION_LENGTH == 2)
			m_Value = 2048;
		else
			m_Value = 0;

		m_DXL_Comm = 0;
	}

	~JointData()
	{	}

};

}

#endif /* JOINTDATA_H_ */
