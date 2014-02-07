/*
 * DXLComm.h
 *
 *  Created on: 2013. 1. 22.
 *      Author: hjsong
 */

#ifndef DXLCOMM_H_
#define DXLCOMM_H_

#include <stdio.h>
#include "dynamixel.h"

namespace Thor
{
class DXLComm
{
private:
	unsigned char txpacket[1000];
	int txpacketlength;
	Dynamixel *m_DXL;

public:
	DXLComm(const char* port_name)
	{
		txpacketlength = 0;
		m_DXL = new Dynamixel(port_name);
	}
	~DXLComm()
	{
		m_DXL->Disconnect();
		delete m_DXL;
	}

	Dynamixel* GetDXLInstance()
	{
		return m_DXL;
	}

	void AddTxpacket(int id, int value)
	{
		txpacket[txpacketlength++] = (unsigned char) id;
		txpacket[txpacketlength++] = DXL_LOBYTE(DXL_LOWORD(value));
		txpacket[txpacketlength++] = DXL_HIBYTE(DXL_LOWORD(value));
		txpacket[txpacketlength++] = DXL_LOBYTE(DXL_HIWORD(value));
		txpacket[txpacketlength++] = DXL_HIBYTE(DXL_HIWORD(value));
	}

	void AddTxpacket(int id, int value, int pgain, int igain, int dgain)
	{
		txpacket[txpacketlength++] = (unsigned char) id;
		txpacket[txpacketlength++] = DXL_LOBYTE(dgain);
		txpacket[txpacketlength++] = DXL_HIBYTE(dgain);
		txpacket[txpacketlength++] = DXL_LOBYTE(igain);
		txpacket[txpacketlength++] = DXL_HIBYTE(igain);
		txpacket[txpacketlength++] = DXL_LOBYTE(pgain);
		txpacket[txpacketlength++] = DXL_HIBYTE(pgain);
		txpacket[txpacketlength++] = DXL_LOBYTE(DXL_LOWORD(value));
		txpacket[txpacketlength++] = DXL_HIBYTE(DXL_LOWORD(value));
		txpacket[txpacketlength++] = DXL_LOBYTE(DXL_HIWORD(value));
		txpacket[txpacketlength++] = DXL_HIBYTE(DXL_HIWORD(value));
	}

	void ClearTxpacket()
	{
		txpacketlength = 0;
	}

	int SendTxpacket()
	{
		if(txpacketlength != 0)
		{
			if(txpacketlength % 5 == 0)
				return m_DXL->SyncWrite(596, 4, txpacket, txpacketlength );
			else if(txpacketlength % 11 == 0)
				return m_DXL->SyncWrite(590, 10, txpacket, txpacketlength );
			else
				return COMM_TXERROR;
		}
		else
			return COMM_TXERROR;
	}
};



}

#endif /* DXLCOMM_H_ */
