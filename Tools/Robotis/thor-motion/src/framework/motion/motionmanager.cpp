/*
 * motionmanager.cpp
 *
 *  Created on: 2013. 1. 11.
 *      Author: zerom
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "motion/motionmanager.h"

using namespace Thor;

MotionManager* MotionManager::UniqueInstance = new MotionManager();

PRO42* m_tempPro42;
PRO54* m_tempPro54;
MX28* m_tempMX28;

#define CurrentWindowSize   50
#define WattWindowSize      50

int wCurrentArray[30][CurrentWindowSize];
int wCurrentMeanValueArray[30];
int dwWattArray[30][WattWindowSize];
int dwWattMeanValueArray[30];

std::vector<int> bChannel0IDlist;
std::vector<int> bChannel1IDlist;
std::vector<int> bChannel2IDlist;
std::vector<int> bChannel3IDlist;
FILE *fpRLFT;
FILE *fpLLFT;

MotionManager::MotionManager()
{
	m_ProcessEnable = false;
	m_Enabled = false;
	m_IsRunning = false;
	m_IsThreadRunning = false;
	DEBUG_PRINT = false;

	m_DXLComm0 = new DXLComm("/dev/ttyUSB0");
	m_DXLComm1 = new DXLComm("/dev/ttyUSB1");
	m_DXLComm2 = new DXLComm("/dev/ttyUSB2");
	m_DXLComm3 = new DXLComm("/dev/ttyUSB3");

	m_BulkRead0 = new BulkRead(m_DXLComm0->GetDXLInstance());
	m_BulkRead1 = new BulkRead(m_DXLComm1->GetDXLInstance());
	m_BulkRead2 = new BulkRead(m_DXLComm2->GetDXLInstance());
	m_BulkRead3 = new BulkRead(m_DXLComm3->GetDXLInstance());

	Thread_ID = 0;
	m_IsTimerRunning = false;
	m_IsTimerStop = false;

	m_EnableList = new EnableList[MotionStatus::MAXIMUM_NUMBER_OF_JOINTS];

	m_tempPro42 = new PRO42();
	m_tempPro54 = new PRO54();
	m_tempMX28 = new MX28();

	for(int index =0; index < MotionStatus::MAXIMUM_NUMBER_OF_JOINTS; index++)
		m_Offset[index] = 0;

	for(int idx = 0; idx < 28 ; idx++)
		wCurrentMeanValueArray[idx] = dwWattMeanValueArray[idx] = 0;
//fp = fopen("Communication Time Result.txt", "w");
//	fp = fopen("FTansFSR.txt", "w");
//	fprintf(fp, "RFx\tRfy\tRFz\tRtx\tRty\tRtz\tLfx\tLfy\tLfz\tLtx\tLty\tLtz\tR1\tR2\tR3\tR4\tL1\tL2\tL3\tL4\tR\tP\tY\n");

//	fp =  fopen("present_pos.txt", "w");
//	fp =  fopen("present_pos.txt", "w");
	fpRLFT = fopen("FTSensorValueRight.txt", "w");
	fpLLFT = fopen("FTSensorValueLeft.txt", "w");

	comm0 = false;
	comm1 = false;
	comm2 = false;
	comm3 = false;
}


MotionManager::~MotionManager()
{
	delete m_tempPro42;
	delete m_tempPro54;
	StopTimer();

	//fclose(fp);
	fclose(fpRLFT);
	fclose(fpLLFT);

	delete[] m_EnableList;
}


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

bool MotionManager::Initialize()
{
	int value = 0;
	int error = 0;

	m_Enabled = false;
	m_ProcessEnable = true;

	m_DXLComm0->GetDXLInstance()->Disconnect();
	m_DXLComm1->GetDXLInstance()->Disconnect();
	m_DXLComm2->GetDXLInstance()->Disconnect();
	m_DXLComm3->GetDXLInstance()->Disconnect();


	if(m_DXLComm0->GetDXLInstance()->Connect() == false)
	{
		if(DEBUG_PRINT == true)
			fprintf(stderr, "Fail to connect ttyUSB0\n");
		return false;
	}

	if(m_DXLComm1->GetDXLInstance()->Connect() == false)
	{
		if(DEBUG_PRINT == true)
			fprintf(stderr, "Fail to connect ttyUSB1\n");
		return false;
	}

	if(m_DXLComm2->GetDXLInstance()->Connect() == false)
	{
		if(DEBUG_PRINT == true)
			fprintf(stderr, "Fail to connect ttyUSB2\n");
		return false;
	}

	if(m_DXLComm3->GetDXLInstance()->Connect() == false)
	{
		if(DEBUG_PRINT == true)
			fprintf(stderr, "Fail to connect ttyUSB3\n");
		return false;
	}

	PingInfo pingResult;

	MotionStatus::m_CurrentJoints.clear();

	for(int id = 1; id < MotionStatus::MAXIMUM_NUMBER_OF_JOINTS; id++)
	{
		if(m_DXLComm0->GetDXLInstance()->Ping(id, &pingResult, 0) == true)
		{
			if(pingResult.ModelNumber == 42 || pingResult.ModelNumber == 51200)
			{
				JointData tempJointData(pingResult.ID, (DXLInfo*)m_tempPro42);
				tempJointData.m_DXL_Comm = m_DXLComm0;
				tempJointData.m_BulkRead = m_BulkRead0;
				MotionStatus::m_CurrentJoints.push_back(tempJointData);
				bChannel0IDlist.push_back(id);
			}
			else if(pingResult.ModelNumber == 54024 || pingResult.ModelNumber == 53768 || pingResult.ModelNumber == 54)
			{
				JointData tempJointData(pingResult.ID, (DXLInfo*)m_tempPro54);
				tempJointData.m_DXL_Comm = m_DXLComm0;
				tempJointData.m_BulkRead = m_BulkRead0;
				MotionStatus::m_CurrentJoints.push_back(tempJointData);
				bChannel0IDlist.push_back(id);
			}
			else if(pingResult.ModelNumber == 54152)
			{
				JointData tempJointData(pingResult.ID, (DXLInfo*)m_tempPro54);
				tempJointData.m_DXL_Comm = m_DXLComm0;
				tempJointData.m_BulkRead = m_BulkRead0;
				MotionStatus::m_CurrentJoints.push_back(tempJointData);
				bChannel0IDlist.push_back(id);
			}
			else
			{
				JointData tempJointData(pingResult.ID, (DXLInfo*)m_tempMX28);
				tempJointData.m_DXL_Comm = m_DXLComm0;
				tempJointData.m_BulkRead = m_BulkRead0;
				MotionStatus::m_CurrentJoints.push_back(tempJointData);
				bChannel0IDlist.push_back(id);
			}
		}
		else if(m_DXLComm1->GetDXLInstance()->Ping(id, &pingResult, 0) == true)
		{
			if(pingResult.ModelNumber == 42 || pingResult.ModelNumber == 51200)
			{
				JointData tempJointData(pingResult.ID, (DXLInfo*)m_tempPro42);
				tempJointData.m_DXL_Comm = m_DXLComm1;
				tempJointData.m_BulkRead = m_BulkRead1;
				MotionStatus::m_CurrentJoints.push_back(tempJointData);
				bChannel1IDlist.push_back(id);
			}
			else if(pingResult.ModelNumber == 54024 || pingResult.ModelNumber == 53768 || pingResult.ModelNumber == 54)
			{
				JointData tempJointData(pingResult.ID, (DXLInfo*)m_tempPro54);
				tempJointData.m_DXL_Comm = m_DXLComm1;
				tempJointData.m_BulkRead = m_BulkRead1;
				MotionStatus::m_CurrentJoints.push_back(tempJointData);
				bChannel1IDlist.push_back(id);
			}
			else if(pingResult.ModelNumber == 54152)
			{
				JointData tempJointData(pingResult.ID, (DXLInfo*)m_tempPro54);
				tempJointData.m_DXL_Comm = m_DXLComm1;
				tempJointData.m_BulkRead = m_BulkRead1;
				MotionStatus::m_CurrentJoints.push_back(tempJointData);
				bChannel1IDlist.push_back(id);
			}
			else
			{
				JointData tempJointData(pingResult.ID, (DXLInfo*)m_tempMX28);
				tempJointData.m_DXL_Comm = m_DXLComm1;
				tempJointData.m_BulkRead = m_BulkRead1;
				MotionStatus::m_CurrentJoints.push_back(tempJointData);
				bChannel1IDlist.push_back(id);
			}
		}
		else if(m_DXLComm2->GetDXLInstance()->Ping(id, &pingResult, 0) == true)
		{
			if(pingResult.ModelNumber == 42 || pingResult.ModelNumber == 51200)
			{
				JointData tempJointData(pingResult.ID, (DXLInfo*)m_tempPro42);
				tempJointData.m_DXL_Comm = m_DXLComm2;
				tempJointData.m_BulkRead = m_BulkRead2;
				MotionStatus::m_CurrentJoints.push_back(tempJointData);
				bChannel2IDlist.push_back(id);
			}
			else if(pingResult.ModelNumber == 54024 || pingResult.ModelNumber == 53768 || pingResult.ModelNumber == 54)
			{
				JointData tempJointData(pingResult.ID, (DXLInfo*)m_tempPro54);
				tempJointData.m_DXL_Comm = m_DXLComm2;
				tempJointData.m_BulkRead = m_BulkRead2;
				MotionStatus::m_CurrentJoints.push_back(tempJointData);
				bChannel2IDlist.push_back(id);
			}
			else if(pingResult.ModelNumber == 54152)
			{
				JointData tempJointData(pingResult.ID, (DXLInfo*)m_tempPro54);
				tempJointData.m_DXL_Comm = m_DXLComm2;
				tempJointData.m_BulkRead = m_BulkRead2;
				MotionStatus::m_CurrentJoints.push_back(tempJointData);
				bChannel2IDlist.push_back(id);
			}
			else
			{
				JointData tempJointData(pingResult.ID, (DXLInfo*)m_tempMX28);
				tempJointData.m_DXL_Comm = m_DXLComm2;
				tempJointData.m_BulkRead = m_BulkRead2;
				MotionStatus::m_CurrentJoints.push_back(tempJointData);
				bChannel2IDlist.push_back(id);
			}
		}
		else if(m_DXLComm3->GetDXLInstance()->Ping(id, &pingResult, 0) == true)
		{
			if(pingResult.ModelNumber == 42 || pingResult.ModelNumber == 51200)
			{
				JointData tempJointData(pingResult.ID, (DXLInfo*)m_tempPro42);
				tempJointData.m_DXL_Comm = m_DXLComm3;
				tempJointData.m_BulkRead = m_BulkRead3;
				MotionStatus::m_CurrentJoints.push_back(tempJointData);
				bChannel3IDlist.push_back(id);
			}
			else if(pingResult.ModelNumber == 54024 || pingResult.ModelNumber == 53768 || pingResult.ModelNumber == 54)
			{
				JointData tempJointData(pingResult.ID, (DXLInfo*)m_tempPro54);
				tempJointData.m_DXL_Comm = m_DXLComm3;
				tempJointData.m_BulkRead = m_BulkRead3;
				MotionStatus::m_CurrentJoints.push_back(tempJointData);
				bChannel3IDlist.push_back(id);
			}
			else if(pingResult.ModelNumber == 54152)
			{
				JointData tempJointData(pingResult.ID, (DXLInfo*)m_tempPro54);
				tempJointData.m_DXL_Comm = m_DXLComm3;
				tempJointData.m_BulkRead = m_BulkRead3;
				MotionStatus::m_CurrentJoints.push_back(tempJointData);
				bChannel3IDlist.push_back(id);
			}
			else
			{
				JointData tempJointData(pingResult.ID, (DXLInfo*)m_tempMX28);
				tempJointData.m_DXL_Comm = m_DXLComm3;
				tempJointData.m_BulkRead = m_BulkRead3;
				MotionStatus::m_CurrentJoints.push_back(tempJointData);
				bChannel3IDlist.push_back(id);
			}
		}
		else
		{
			fprintf(stderr, "id %d scan fail\n", id);
			//return false;
		}
		usleep(2000);
	}

	for(unsigned int index = 0; index < MotionStatus::m_CurrentJoints.size(); index++)
	{
		int id = MotionStatus::m_CurrentJoints[index].m_ID;
		if(MotionStatus::m_CurrentJoints[index].m_DXLInfo->POSITION_LENGTH == 2)
		{
			MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteWord(id, 30, 2048, &error);
			usleep(10000);
		}
		else
			MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteByte(id, PRO54::P_TORQUE_ENABLE, 1, &error);

		usleep(10000);
	}


    m_DXLComm0->ClearTxpacket();
    m_DXLComm1->ClearTxpacket();
    m_DXLComm2->ClearTxpacket();
    m_DXLComm3->ClearTxpacket();

    m_BulkRead0->ClearBulkReadData();
    m_BulkRead1->ClearBulkReadData();
    m_BulkRead2->ClearBulkReadData();
    m_BulkRead3->ClearBulkReadData();

    //setting bulkread
	for(unsigned int index = 0; index < MotionStatus::m_CurrentJoints.size(); index++)
	{
		int id = MotionStatus::m_CurrentJoints[index].m_ID;

//		if( 0 < id && id < 29 )
//		{

			//			if( id == 7 || id == 8 || id == 11 || id == 12 || id == 19 || id == 20 || id ==23 || id == 24)
			//				MotionStatus::m_CurrentJoints[index].m_BulkRead->AddBulkReadData(id, PRO54::P_INDIRECT_DATA_0, 18);
			//			else if(id == 9 || id == 10 || id == 21 || id == 22)
			//				MotionStatus::m_CurrentJoints[index].m_BulkRead->AddBulkReadData(id, PRO54::P_INDIRECT_DATA_0, 14);
			//			else
			//				MotionStatus::m_CurrentJoints[index].m_BulkRead->AddBulkReadData(id, PRO54::P_INDIRECT_DATA_0, 10);
//			if( id == 15 || id == 16)
//			{
//				MotionStatus::m_CurrentJoints[index].m_BulkRead->AddBulkReadData(id, PRO54::P_PRESENT_POSITION_LL, 4);
//			}
//
//			if(id == 19 || id == 20 || id == 23 || id == 24)
//			{
//				MotionStatus::m_CurrentJoints[index].m_BulkRead->AddBulkReadData(id, PRO54::P_EXTERNAL_PORT_DATA_1_L, 8);
//			}
//
//			if(id == 21 || id == 22 )
//			{
//				MotionStatus::m_CurrentJoints[index].m_BulkRead->AddBulkReadData(id, PRO54::P_EXTERNAL_PORT_DATA_1_L, 4);
//			}


//		}
		//		else if(28 < id && id < 36)
		//		{
		//			MotionStatus::m_CurrentJoints[index].m_BulkRead->AddBulkReadData(id, MX28::P_PRESENT_POSITION_L, 2);
//		}

//		if(id == 20)
//			MotionStatus::m_CurrentJoints[index].m_BulkRead->AddBulkReadData(id, PRO54::P_PRESENT_POSITION_LL, 8);
//		if(id == 22)
//			MotionStatus::m_CurrentJoints[index].m_BulkRead->AddBulkReadData(id, PRO54::P_PRESENT_POSITION_LL, 8);
//		if(id == 7 || id == 8 || id ==11 || id ==12)
//		{
//			MotionStatus::m_CurrentJoints[index].m_BulkRead->AddBulkReadData(id, PRO54::P_EXTERNAL_PORT_DATA_1_L, 8);
//		}
//		if(id == 9 || id ==10 )
//		{
//			MotionStatus::m_CurrentJoints[index].m_BulkRead->AddBulkReadData(id, PRO54::P_EXTERNAL_PORT_DATA_1_L, 4);
//		}
//
//		if(id == 19 || id == 20 || id == 23 || id == 24)
//			MotionStatus::m_CurrentJoints[index].m_BulkRead->AddBulkReadData(id, PRO54::P_EXTERNAL_PORT_DATA_1_L, 8);
//
//		if(id == 21 || id == 22 )
//		{
//			MotionStatus::m_CurrentJoints[index].m_BulkRead->AddBulkReadData(id, PRO54::P_EXTERNAL_PORT_DATA_1_L, 4);
//		}
		if( id == 25 || id == 26)
			MotionStatus::m_CurrentJoints[index].m_BulkRead->AddBulkReadData(id, PRO54::P_EXTERNAL_PORT_DATA_1_L, 8);

		if(id == 23 || id == 24 )
		{
			MotionStatus::m_CurrentJoints[index].m_BulkRead->AddBulkReadData(id, PRO54::P_EXTERNAL_PORT_DATA_1_L, 4);
		}

	}

	for(unsigned int index = 0; index < MotionStatus::m_CurrentJoints.size(); index++)
	{
		int id = MotionStatus::m_CurrentJoints[index].m_ID;
		int position_length = MotionStatus::m_CurrentJoints[index].m_DXLInfo->POSITION_LENGTH;
		int present_position_addr = MotionStatus::m_CurrentJoints[index].m_DXLInfo->PRESENT_POSITION_ADDR;
		if(position_length == 4)
		{
			MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->ReadDWord(id, present_position_addr, (long*) &value, &error);
			if(abs(value) > MotionStatus::m_CurrentJoints[index].m_DXLInfo->MAX_VALUE)
			{
				fprintf(stderr, "id %d read fail : %d\n", id, value);
				return false;
			}
		}
		else if(position_length == 2)
		{
			MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->ReadWord(id, present_position_addr, &value, &error);
			if(abs(value) > MotionStatus::m_CurrentJoints[index].m_DXLInfo->MAX_VALUE)
			{
				fprintf(stderr, "id %d read fail : %d\n", id, value);
				return false;
			}
		}
		else
		{
			fprintf(stderr, "id %d read fail\n", id);
			//return false;
		}
		MotionStatus::m_CurrentJoints[index].m_Value = value;
		MotionStatus::m_CurrentJoints[index].m_Angle = MotionStatus::m_CurrentJoints[index].m_DXLInfo->Value2Angle(value);
	}

	fprintf(stderr, "mesurement current start\n");

//  Initialize Current and Watt
//	for(int i = 0 ; i < CurrentWindowSize ; i ++)
//	{
//		printf("%d\n", i);
//		int comm_result0 = m_BulkRead0->SendTxPacket();
//		int comm_result1 = m_BulkRead1->SendTxPacket();
//		int comm_result2 = m_BulkRead2->SendTxPacket();
//		int comm_result3 = m_BulkRead3->SendTxPacket();
//
//		if(comm_result0 != COMM_RXSUCCESS || comm_result1 != COMM_RXSUCCESS || comm_result2 != COMM_RXSUCCESS || comm_result3 != COMM_RXSUCCESS)
//		{
//			i--;
//			continue;
//		}
//
//		int wCurrentMeanValueArray[28];
//		int dwWattMeanValueArray[28];
//
//		for(unsigned int idx = 0; idx < bChannel0IDlist.size(); idx++)
//		{
//			int id = bChannel0IDlist[idx];
//			short wCurrent_Value;
//			int dwWatt_value;
//
//			m_BulkRead0->GetWordValue(id, PRO54::P_INDIRECT_DATA_4, (int*)&wCurrent_Value);
//			wCurrentArray[id -1][i] = abs(wCurrent_Value);
//
//			m_BulkRead0->GetDwordValue(id, PRO54::P_INDIRECT_DATA_6, (long*)&dwWatt_value);
//			dwWattArray[id - 1][i] = abs(dwWatt_value);
//		}
//
//		for(unsigned int idx = 0; idx < bChannel1IDlist.size(); idx++)
//		{
//			int id = bChannel1IDlist[idx];
//			short wCurrent_Value;
//			int dwWatt_value;
//
//			m_BulkRead1->GetWordValue(id, PRO54::P_INDIRECT_DATA_4, (int*)&wCurrent_Value);
//			wCurrentArray[id -1][i] = abs(wCurrent_Value);
//
//			m_BulkRead1->GetDwordValue(id, PRO54::P_INDIRECT_DATA_6, (long*)&dwWatt_value);
//			dwWattArray[id - 1][i] = abs(dwWatt_value);
//		}
//
//		for(unsigned int idx = 0; idx < bChannel2IDlist.size(); idx++)
//		{
//			int id = bChannel2IDlist[idx];
//			short wCurrent_Value;
//			int dwWatt_value;
//
//			m_BulkRead2->GetWordValue(id, PRO54::P_INDIRECT_DATA_4, (int*)&wCurrent_Value);
//			wCurrentArray[id -1][i] = abs(wCurrent_Value);
//
//			m_BulkRead2->GetDwordValue(id, PRO54::P_INDIRECT_DATA_6, (long*)&dwWatt_value);
//			dwWattArray[id - 1][i] = abs(dwWatt_value);
//		}
//
//		for(unsigned int idx = 0; idx < bChannel3IDlist.size(); idx++)
//		{
//			int id = bChannel3IDlist[idx];
//			short wCurrent_Value;
//			int dwWatt_value;
//
//			m_BulkRead3->GetWordValue(id, PRO54::P_INDIRECT_DATA_4, (int*)&wCurrent_Value);
//			wCurrentArray[id -1][i] = abs(wCurrent_Value);
//
//			m_BulkRead3->GetDwordValue(id, PRO54::P_INDIRECT_DATA_6, (long*)&dwWatt_value);
//			dwWattArray[id - 1][i] = abs(dwWatt_value);
//		}
//	}
//
//	fprintf(stderr, "Current Measurement Complete1\n");
//	for(int i = 0; i < CurrentWindowSize; i++)
//	{
//		for(int idx = 0; idx < 28; idx++)
//		{
//			wCurrentMeanValueArray[idx] += wCurrentArray[idx][i];
//			dwWattMeanValueArray[idx] += dwWattArray[idx][i];
//		}
//	}
//
//	fprintf(stderr, "Current Measurement Complete2\n");
//	for(int idx = 0; idx < 28; idx++)
//	{
//		wCurrentMeanValueArray[idx] = ((double)wCurrentMeanValueArray[idx]/(double)CurrentWindowSize);
//		dwWattMeanValueArray[idx] = ((double)dwWattMeanValueArray[idx]/(double)WattWindowSize);
//
//		fprintf(stderr,"id : %d   Current : %d\n", idx+1, wCurrentMeanValueArray[idx]);
//		fprintf(stderr,"id : %d   Watt    : %d\n", idx+1, dwWattMeanValueArray[idx]);
//	}


	fprintf(stderr, "Current Measurement Complete\n");


	m_CalibrationStatus = 0;
	m_FBGyroCenter = 512;
	m_RLGyroCenter = 512;

	return true;
}

bool MotionManager::Reinitialize()
{
	m_ProcessEnable = false;

	Initialize();

	m_ProcessEnable = true;
	return true;
}

bool MotionManager::InitforTest()
{

	for(unsigned int index = 0; index < MotionStatus::m_CurrentJoints.size(); index++)
	{
		int id = MotionStatus::m_CurrentJoints[index].m_ID;
		if(id != 27 && id != 28)
			MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteDWord(id, PRO54::P_GOAL_VELOCITY_LL, 17000, 0);
		else
			MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteDWord(id, PRO54::P_GOAL_VELOCITY_LL, 10000, 0);
	}

	for(unsigned int index = 0; index < MotionStatus::m_CurrentJoints.size(); index++)
	{
		int id = MotionStatus::m_CurrentJoints[index].m_ID;
		int value = 0;
		MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->ReadDWord(id, PRO54::P_GOAL_VELOCITY_LL, (long*) &value, 0);

		if(id != 27 && id != 28)
		{
			if(value != 17000)
				return false;
		}
		else
		{
			if(value != 10000)
				return false;
		}

	}

	for(unsigned int index = 0; index < MotionStatus::m_CurrentJoints.size(); index++)
	{
		int id = MotionStatus::m_CurrentJoints[index].m_ID;
		if(id != 27 && id != 28)
			MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteDWord(id, PRO54::P_GOAL_ACCELATION_LL, 200, 0);
		else
			MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteDWord(id, PRO54::P_GOAL_ACCELATION_LL, 100, 0);


	}

	for(unsigned int index = 0; index < MotionStatus::m_CurrentJoints.size(); index++)
	{
		int id = MotionStatus::m_CurrentJoints[index].m_ID;
		int value = 0;
		MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->ReadDWord(id, PRO54::P_GOAL_ACCELATION_LL, (long*) &value, 0);
		if(id != 27 && id != 28)
		{
			if(value != 200)
				return false;
		}
		else
		{
			if(value != 100)
				return false;
		}
	}

	return true;
}

void MotionManager::AddModule(MotionModule *module)
{
	module->Initialize();
	m_Modules.push_back(module);
}

void MotionManager::RemoveModule(MotionModule *module)
{
	m_Modules.remove(module);
}

void MotionManager::StartLogging()
{
    char szFile[32] = {0,};

    int count = 0;
    while(1)
    {
        sprintf(szFile, "Logs/Log%d.csv", count);
        if(0 != access(szFile, F_OK))
            break;
        count++;
		if(count > 256) return;
    }

    m_LogFileStream.open(szFile, std::ios::out);
    for(int i = 0; i < MotionStatus::m_CurrentJoints.size(); i++)
        m_LogFileStream << "ID_" << MotionStatus::m_CurrentJoints[i].m_ID << "_GP,ID_" << MotionStatus::m_CurrentJoints[i].m_ID << "_PP,";
    m_LogFileStream << "GyroFB,GyroRL,AccelFB,AccelRL,L_FSR_X,L_FSR_Y,R_FSR_X,R_FSR_Y" << std::endl;

    m_IsLogging = true;
}

bool IsLogging = false;
void MotionManager::TempStartLogging()
{

	IsLogging = true;
}

void MotionManager::TempStopLogging()
{

	IsLogging = false;
}

void MotionManager::StopLogging()
{
    m_IsLogging = false;
    m_LogFileStream.close();
}

void MotionManager::LoadINISettings(minIni* ini)
{
	LoadINISettings(ini, OFFSET_SECTION);
	RightArmFTSensor.LoadINISettings(ini, "Right Arm FT Sensor");
	LeftArmFTSensor.LoadINISettings(ini, "Left Arm FT Sensor");
	RightLegFTSensor.LoadINISettings(ini, "Right Leg FT Sensor");
	LeftLegFTSensor.LoadINISettings(ini, "Left Leg FT Sensor");
}

void MotionManager::LoadINISettings(minIni* ini, const std::string &section)
{
    int ivalue = INVALID_VALUE;

    for(int i = 0; i < MotionStatus::m_CurrentJoints.size(); i++)
    {
    	int id = MotionStatus::m_CurrentJoints[i].m_ID;
        char key[10];
        sprintf(key, "ID_%.2d", id);
        if((ivalue = ini->geti(section, key, INVALID_VALUE)) != INVALID_VALUE)  m_Offset[id-1] = ivalue;
    }
}

void MotionManager::SaveINISettings(minIni* ini)
{
	SaveINISettings(ini, OFFSET_SECTION);
}

void MotionManager::SaveINISettings(minIni* ini, const std::string &section)
{
    for(int i = 0; i <MotionStatus::m_CurrentJoints.size(); i++)
    {
    	int id = MotionStatus::m_CurrentJoints[i].m_ID;
        char key[10];
        sprintf(key, "ID_%.2d", id);
        ini->put(section, key, m_Offset[id-1]);
    }
}

int MotionManager::WriteByte(int id, int addr, int value, int* error)
{
	for(unsigned int jointIndex = 0; jointIndex<MotionStatus::m_CurrentJoints.size(); jointIndex++)
	{
		if(MotionStatus::m_CurrentJoints[jointIndex].m_ID == id)
		{
			return MotionStatus::m_CurrentJoints[jointIndex].m_DXL_Comm->GetDXLInstance()->WriteByte(id, addr, value, error);
		}
	}

	return COMM_TXFAIL;
}

int MotionManager::WriteWord(int id, int addr, int value, int* error)
{
	for(unsigned int jointIndex = 0; jointIndex<MotionStatus::m_CurrentJoints.size(); jointIndex++)
	{
		if(MotionStatus::m_CurrentJoints[jointIndex].m_ID == id)
		{
			return MotionStatus::m_CurrentJoints[jointIndex].m_DXL_Comm->GetDXLInstance()->WriteWord(id, addr, value, error);
		}
	}
	return COMM_TXFAIL;
}

int MotionManager::WriteDWord(int id, int addr, long value, int* error)
{
	for(unsigned int jointIndex = 0; jointIndex<MotionStatus::m_CurrentJoints.size(); jointIndex++)
	{
		if(MotionStatus::m_CurrentJoints[jointIndex].m_ID == id)
		{
			return MotionStatus::m_CurrentJoints[jointIndex].m_DXL_Comm->GetDXLInstance()->WriteDWord(id, addr, value, error);
		}
	}
	return COMM_TXFAIL;
}

int MotionManager::ReadByte(int id, int addr, int* value, int* error)
{
	for(unsigned int jointIndex = 0; jointIndex<MotionStatus::m_CurrentJoints.size(); jointIndex++)
	{
		if(MotionStatus::m_CurrentJoints[jointIndex].m_ID == id)
		{
			return MotionStatus::m_CurrentJoints[jointIndex].m_DXL_Comm->GetDXLInstance()->ReadByte(id, addr, value, error);
		}
	}
	return COMM_TXFAIL;
}

int MotionManager::ReadWord(int id, int addr, int* value, int* error)
{
	for(unsigned int jointIndex = 0; jointIndex<MotionStatus::m_CurrentJoints.size(); jointIndex++)
	{
		if(MotionStatus::m_CurrentJoints[jointIndex].m_ID == id)
		{
			return MotionStatus::m_CurrentJoints[jointIndex].m_DXL_Comm->GetDXLInstance()->ReadWord(id, addr, value, error);
		}
	}
	return COMM_TXFAIL;
}

int MotionManager::ReadDWord(int id, int addr, long* value, int* error)
{
	for(unsigned int jointIndex = 0; jointIndex<MotionStatus::m_CurrentJoints.size(); jointIndex++)
	{
		if(MotionStatus::m_CurrentJoints[jointIndex].m_ID == id)
		{
			return MotionStatus::m_CurrentJoints[jointIndex].m_DXL_Comm->GetDXLInstance()->ReadDWord(id, addr, value, error);
		}
	}
	return COMM_TXFAIL;
}

void MotionManager::CheckFTSensorSetting()
{
	fprintf(stderr, "\nRight Arm FT Sensor\n");
	RightArmFTSensor.CheckSensorSetting();

	fprintf(stderr, "\nLeft Arm FT Sensor\n");
	LeftArmFTSensor.CheckSensorSetting();

	fprintf(stderr, "\nRight Leg FT Sensor\n");
	RightLegFTSensor.CheckSensorSetting();

	fprintf(stderr, "\nLeft Leg FT Sensor\n");
	LeftLegFTSensor.CheckSensorSetting();
}

void MotionManager::SetEnable()
{
	for(int i = 0; i < 36; i++)
	{
		m_EnableList[i].uID = MotionStatus::m_EnableList[i].uID;
	}
}


#define GYRO_WINDOW_SIZE    100
#define ACCEL_WINDOW_SIZE   30
#define MARGIN_OF_SD        2.0

static int present_pos_15 = 0;
static int present_pos_16 = 0;

void MotionManager::Process()
{
    if(m_ProcessEnable == false || m_IsRunning == true)
        return;

    m_IsRunning = true;
    int val;
    // calibrate gyro sensor


    if(m_Modules.size() != 0)
    {
    	int id = -1;
    	for(std::list<MotionModule*>::iterator i = m_Modules.begin(); i != m_Modules.end(); i++)
    	{
    		(*i)->Process();
    		for(unsigned int jointIndex=0; jointIndex<MotionStatus::m_CurrentJoints.size(); jointIndex++)
    		{
    			id = MotionStatus::m_CurrentJoints[jointIndex].m_ID;
    			if( MotionStatus::m_EnableList[id-1].IsEqual((*i)->uID) )
    			{
    				MotionStatus::m_CurrentJoints[jointIndex].m_Value = (*i)->m_RobotInfo[jointIndex].m_Value + m_Offset[id-1];
    				MotionStatus::m_CurrentJoints[jointIndex].m_Angle = (*i)->m_RobotInfo[jointIndex].m_Angle;
    				MotionStatus::m_CurrentJoints[jointIndex].m_Pgain = (*i)->m_RobotInfo[jointIndex].m_Pgain;
    				MotionStatus::m_CurrentJoints[jointIndex].m_Igain = (*i)->m_RobotInfo[jointIndex].m_Igain;
    				MotionStatus::m_CurrentJoints[jointIndex].m_Dgain = (*i)->m_RobotInfo[jointIndex].m_Dgain;
    			}
    		}
    	}

    	for(unsigned int jointIndex=0; jointIndex<MotionStatus::m_CurrentJoints.size(); jointIndex++)
    	{
    		id = MotionStatus::m_CurrentJoints[jointIndex].m_ID;

    		if( MotionStatus::m_CurrentJoints[jointIndex].m_DXLInfo->POSITION_LENGTH == 4 )
    		{
    			MotionStatus::m_CurrentJoints[jointIndex].m_DXL_Comm->AddTxpacket(MotionStatus::m_CurrentJoints[jointIndex].m_ID,
    					MotionStatus::m_CurrentJoints[jointIndex].m_Value,
    					MotionStatus::m_CurrentJoints[jointIndex].m_Pgain,
    					MotionStatus::m_CurrentJoints[jointIndex].m_Igain,
    					MotionStatus::m_CurrentJoints[jointIndex].m_Dgain);
    		}
    		//    		if(id == 15)
//    			present_pos_15 = MotionStatus::m_CurrentJoints[jointIndex].m_Value;
//
//    		if(id == 16)
//    			present_pos_16 = MotionStatus::m_CurrentJoints[jointIndex].m_Value;

    	}


    		//        	if(MotionStatus::m_CurrentJoints[jointIndex].m_DXLInfo->MODEL_NUM != 28)
////        	{
//    		if(id == 17 && id == 18 && id ==21 && id == 22)
//    		{
//        		MotionStatus::m_CurrentJoints[jointIndex].m_DXL_Comm->
//        		AddTxpacket(MotionStatus::m_CurrentJoints[jointIndex].m_ID,
//        				MotionStatus::m_CurrentJoints[jointIndex].m_Value,
//        				MotionStatus::m_CurrentJoints[jointIndex].m_Pgain*1.5,
//        				MotionStatus::m_CurrentJoints[jointIndex].m_Igain,
//        				MotionStatus::m_CurrentJoints[jointIndex].m_Dgain);
//    		}
//    		else
//    		{
//        		MotionStatus::m_CurrentJoints[jointIndex].m_DXL_Comm->
//        		AddTxpacket(MotionStatus::m_CurrentJoints[jointIndex].m_ID,
//        				MotionStatus::m_CurrentJoints[jointIndex].m_Value,
//        				MotionStatus::m_CurrentJoints[jointIndex].m_Pgain,
//        				MotionStatus::m_CurrentJoints[jointIndex].m_Igain,
//        				MotionStatus::m_CurrentJoints[jointIndex].m_Dgain);
//    		}
//        	}
 //       }

//    	static struct timespec StartTime, EndTime;
//    	clock_gettime(CLOCK_REALTIME , &StartTime);
//

    	if(pthread_create(&this->m_DXLCommThred0, 0, this->DXLCommProc0, this) != 0)
    		exit(-1);

    	if(pthread_create(&this->m_DXLCommThred1, 0, this->DXLCommProc1, this) != 0)
    		exit(-1);

    	if(pthread_create(&this->m_DXLCommThred2, 0, this->DXLCommProc2, this) != 0)
    		exit(-1);

    	if(pthread_create(&this->m_DXLCommThred3, 0, this->DXLCommProc3, this) != 0)
    		exit(-1);


    	pthread_join(m_DXLCommThred0, 0);
    	pthread_join(m_DXLCommThred1, 0);
    	pthread_join(m_DXLCommThred2, 0);
    	pthread_join(m_DXLCommThred3, 0);

    	if(IsLogging)
    	{
//    		fprintf(fp, "%f\t %f\t %f\t %f\t %f\t %f\t %f\t %f\t %f\t %f\t %f\t %f\t %f\t %f\t %f\t %f\t %f\t %f\t %f\t %f\t %f\t %f\t %f\t\n",
//    				MotionStatus::R_LEG_FX,
//    				MotionStatus::R_LEG_FY,
//    				MotionStatus::R_LEG_FZ,
//    				MotionStatus::R_LEG_TX,
//    				MotionStatus::R_LEG_TY,
//    				MotionStatus::R_LEG_TZ,
//    				MotionStatus::L_LEG_FX,
//    				MotionStatus::L_LEG_FY,
//      				MotionStatus::L_LEG_FZ,
//       				MotionStatus::L_LEG_TX,
//     				MotionStatus::L_LEG_TY,
//     				MotionStatus::L_LEG_TZ,
//     				MotionStatus::R_LEG_FSR1,
//     				MotionStatus::R_LEG_FSR2,
//     				MotionStatus::R_LEG_FSR3,
//     				MotionStatus::R_LEG_FSR4,
//     				MotionStatus::L_LEG_FSR1,
//     				MotionStatus::L_LEG_FSR2,
//     				MotionStatus::L_LEG_FSR3,
//     				MotionStatus::L_LEG_FSR4,
//     				MotionStatus::EulerAngleX,
//     				MotionStatus::EulerAngleY,
//     				MotionStatus::EulerAngleZ);
    	}
    	//fprintf(fp, "%d\t%d\n", present_pos_15, present_pos_16);
//
//    	fprintf(stderr,"%d\t%d\n", present_pos_15, present_pos_16);

//    	clock_gettime(CLOCK_REALTIME , &EndTime);
//
//    	double time_dif = (EndTime.tv_sec - StartTime.tv_sec)*1000000000 + ((EndTime.tv_nsec - StartTime.tv_nsec) );
//    	time_dif = time_dif/ 1000000000;
//       	fprintf(fp, "%f\n", time_dif);
//    	printf(" %f\n",time_dif);

    }

    m_IsRunning = false;
}

//static int wRightArmFTVoltageOutput[6];
//static int wLeftArmFTVoltageOutput[6];
//
static int wRightLegFTVoltageOutput[6];
static int wLeftLegFTVoltageOutput[6];
//
//static int wRightArmFSRVoltageOutput[4];
//static int wLeftArmFSRVoltageOutput[4];
//
//static int wRightLegFSRVoltageOutput[4];
//static int wLeftLegFSRVoltageOutput[4];
//
//static int result0;
//static int result1;
//static int result2;
//static int result3;
//static int debug_num = 0;
//
//static int present_position0;
//static int present_position1;
//static int present_position2;
//static int present_position3;
//
//static int present_current0;
//static int present_current1;
//static int present_current2;
//static int present_current3;
//
//static int present_watt0;
//static int present_watt1;
//static int present_watt2;
//static int present_watt3;

unsigned char param0[15];
void* MotionManager::DXLCommProc0(void *param)
{
	//debug_num +=1;
	MotionManager* manager = (MotionManager*)param;
	if(manager->comm0)
		return 0;
	manager->m_DXLComm0->SendTxpacket();
	manager->m_DXLComm0->ClearTxpacket();

	param0[0]  = 31;
	param0[1]  = (MotionStatus::m_CurrentJoints[30].m_Pgain);

	param0[2]  = 33;
	param0[3]  = (MotionStatus::m_CurrentJoints[32].m_Pgain);

	param0[4] = 35;
	param0[5] = (MotionStatus::m_CurrentJoints[34].m_Pgain);

	manager->m_DXLComm0->GetDXLInstance()->SyncWrite(28, 1, param0, 6);


	param0[0]  = 31;
	param0[1]  = DXL_LOBYTE(MotionStatus::m_CurrentJoints[30].m_Value);
	param0[2]  = DXL_HIBYTE(MotionStatus::m_CurrentJoints[30].m_Value);

	param0[3]  = 33;
	param0[4]  = DXL_LOBYTE(MotionStatus::m_CurrentJoints[32].m_Value);
	param0[5]  = DXL_HIBYTE(MotionStatus::m_CurrentJoints[32].m_Value);

	param0[6]  = 35;
	param0[7]  = DXL_LOBYTE(MotionStatus::m_CurrentJoints[34].m_Value);
	param0[8]  = DXL_HIBYTE(MotionStatus::m_CurrentJoints[34].m_Value);

	manager->m_DXLComm0->GetDXLInstance()->SyncWrite(30, 2, param0, 9);

//	result0 = manager->m_BulkRead0->SendTxPacket();
//
//	static int current_idx = 0;
//	static int watt_idx = 0;
//
//
//	if(result0 != COMM_RXSUCCESS)
//	{
//		//printf("result : %d   num : %d\n", result0, debug_num);
//		//debug_num = 0;
//		return &result0;
//	}
//
//	current_idx += 1;
//	watt_idx += 1;
//
//	if(current_idx == CurrentWindowSize)
//		current_idx = 0;
//
//	if(watt_idx == WattWindowSize)
//		watt_idx = 0;
//
//	for(unsigned int idx = 0; idx < bChannel0IDlist.size() ; idx++)
//	{
//		manager->m_BulkRead0->GetDwordValue(bChannel0IDlist[idx], PRO54::P_INDIRECT_DATA_0, (long*)&present_position0);
//		manager->m_BulkRead0->GetWordValue(bChannel0IDlist[idx], PRO54::P_INDIRECT_DATA_4, &present_current0);
//		manager->m_BulkRead0->GetDwordValue(bChannel0IDlist[idx], PRO54::P_INDIRECT_DATA_6, (long*)&present_watt0);
//
//		wCurrentMeanValueArray[bChannel0IDlist[idx] - 1] = (double)(wCurrentMeanValueArray[bChannel0IDlist[idx] - 1]*CurrentWindowSize - wCurrentArray[bChannel0IDlist[idx] - 1][current_idx] + present_current0 )/(double)CurrentWindowSize;
//		wCurrentArray[bChannel0IDlist[idx] - 1][current_idx] = present_current0;
//
//		dwWattMeanValueArray[bChannel0IDlist[idx] - 1] = (double)(dwWattMeanValueArray[bChannel0IDlist[idx] - 1]*WattWindowSize - dwWattArray[bChannel0IDlist[idx] - 1][watt_idx] + present_watt0 )/(double)WattWindowSize;
//		dwWattArray[bChannel0IDlist[idx] - 1][watt_idx] = present_watt0;
//	}
//
//	static int FSR1, FSR2,FSR3,FSR4;
//
////	manager->m_BulkRead0->GetWordValue(24, PRO54::P_INDIRECT_DATA_10, &(wLeftLegFTVoltageOutput[0]) );
////	manager->m_BulkRead0->GetWordValue(24, PRO54::P_INDIRECT_DATA_12, &(wLeftLegFTVoltageOutput[1]) );
////	manager->m_BulkRead0->GetWordValue(24, PRO54::P_INDIRECT_DATA_14, &(wLeftLegFTVoltageOutput[2]) );
////	manager->m_BulkRead0->GetWordValue(24, PRO54::P_INDIRECT_DATA_16, &(wLeftLegFTVoltageOutput[3]) );
////
////	manager->m_BulkRead0->GetWordValue(22, PRO54::P_INDIRECT_DATA_10, &(wLeftLegFTVoltageOutput[4]) );
////	manager->m_BulkRead0->GetWordValue(22, PRO54::P_INDIRECT_DATA_12, &(wLeftLegFTVoltageOutput[5]) );
//	manager->m_BulkRead0->GetDwordValue(16, PRO54::P_PRESENT_POSITION_LL, (long*) &(present_pos_16));
//	manager->m_BulkRead0->GetWordValue(24, PRO54::P_EXTERNAL_PORT_DATA_1_L, &(wLeftLegFTVoltageOutput[0]) );
//	manager->m_BulkRead0->GetWordValue(24, PRO54::P_EXTERNAL_PORT_DATA_2_L, &(wLeftLegFTVoltageOutput[1]) );
//	manager->m_BulkRead0->GetWordValue(24, PRO54::P_EXTERNAL_PORT_DATA_3_L, &(wLeftLegFTVoltageOutput[2]) );
//	manager->m_BulkRead0->GetWordValue(24, PRO54::P_EXTERNAL_PORT_DATA_4_L, &(wLeftLegFTVoltageOutput[3]) );
//
//	manager->m_BulkRead0->GetWordValue(22, PRO54::P_EXTERNAL_PORT_DATA_1_L, &(wLeftLegFTVoltageOutput[4]) );
//	manager->m_BulkRead0->GetWordValue(22, PRO54::P_EXTERNAL_PORT_DATA_2_L, &(wLeftLegFTVoltageOutput[5]) );
//
//	manager->m_BulkRead0->GetWordValue(20, PRO54::P_EXTERNAL_PORT_DATA_1_L, &(FSR1) );
//	manager->m_BulkRead0->GetWordValue(20, PRO54::P_EXTERNAL_PORT_DATA_2_L, &(FSR2) );
//	manager->m_BulkRead0->GetWordValue(20, PRO54::P_EXTERNAL_PORT_DATA_3_L, &(FSR3) );
//	manager->m_BulkRead0->GetWordValue(20, PRO54::P_EXTERNAL_PORT_DATA_4_L, &(FSR4) );
//
//	MotionStatus::L_LEG_FSR1 = FSR1;
//	MotionStatus::L_LEG_FSR2 = FSR2;
//	MotionStatus::L_LEG_FSR3 = FSR3;
//	MotionStatus::L_LEG_FSR4 = FSR4;
//
//	//manager->m_BulkRead0->GetDwordValue(14, PRO54::P_PRESENT_POSITION_LL,
//	//manager->m_BulkRead0->GetWordValue(14, PRO54::P_PRESENT_CURRENT_L,  )
//
//
////	printf("%d %d %d %d %d %d\n",wLeftLegFTVoltageOutput[0],
////								 wLeftLegFTVoltageOutput[1],
////								 wLeftLegFTVoltageOutput[2],
////								 wLeftLegFTVoltageOutput[3],
////								 wLeftLegFTVoltageOutput[4],
////								 wLeftLegFTVoltageOutput[5]);
//	manager->LeftLegFTSensor.setCurrentVoltageOutPut((double)wLeftLegFTVoltageOutput[0]*3.3/4095.0,
//			(double) wLeftLegFTVoltageOutput[1]*3.3/4095.0,
//			(double) wLeftLegFTVoltageOutput[2]*3.3/4095.0,
//			(double) wLeftLegFTVoltageOutput[3]*3.3/4095.0,
//			(double) wLeftLegFTVoltageOutput[4]*3.3/4095.0,
//			(double) wLeftLegFTVoltageOutput[5]*3.3/4095.0 );
//	manager->LeftLegFTSensor.getForceTorque( &(MotionStatus::L_LEG_FX),
//											 &(MotionStatus::L_LEG_FY),
//											 &(MotionStatus::L_LEG_FZ),
//											 &(MotionStatus::L_LEG_TX),
//											 &(MotionStatus::L_LEG_TY),
//											 &(MotionStatus::L_LEG_TZ));

//	int val;
//	manager->m_BulkRead0->GetDwordValue(20, 611, (long*)&val);
//	printf("%d", val);
//	manager->m_BulkRead0->GetDwordValue(22, 611, (long*)&val);
//	printf("  %d\n", val);

}


unsigned char param1[15];
void* MotionManager::DXLCommProc1(void *param)
{
	MotionManager* manager = (MotionManager*)param;
	if(manager->comm1)
		return 0;


	manager->m_DXLComm1->SendTxpacket();
	manager->m_DXLComm1->ClearTxpacket();


//	param1[0]  = 32;
//	param1[1]  = DXL_LOBYTE(MotionStatus::m_CurrentJoints[31].m_Pgain);
//	param1[2]  = DXL_HIBYTE(MotionStatus::m_CurrentJoints[31].m_Pgain);
//	param1[3]  = DXL_LOBYTE(MotionStatus::m_CurrentJoints[31].m_Value);
//	param1[4]  = DXL_HIBYTE(MotionStatus::m_CurrentJoints[31].m_Value);
//	param1[5]  = 34;
//	param1[6]  = DXL_LOBYTE(MotionStatus::m_CurrentJoints[33].m_Pgain);
//	param1[7]  = DXL_HIBYTE(MotionStatus::m_CurrentJoints[33].m_Pgain);
//	param1[8]  = DXL_LOBYTE(MotionStatus::m_CurrentJoints[33].m_Value);
//	param1[9]  = DXL_HIBYTE(MotionStatus::m_CurrentJoints[33].m_Value);
//	param1[10]  = 36;
//	param1[11]  = DXL_LOBYTE(MotionStatus::m_CurrentJoints[35].m_Pgain);
//	param1[12]  = DXL_HIBYTE(MotionStatus::m_CurrentJoints[35].m_Pgain);
//	param1[13] = DXL_LOBYTE(MotionStatus::m_CurrentJoints[35].m_Value);
//	param1[14] = DXL_HIBYTE(MotionStatus::m_CurrentJoints[35].m_Value);
//
//	manager->m_DXLComm1->GetDXLInstance()->SyncWrite(28, 2, param1, 15);

	param1[0]  = 32;
	param1[1]  = (MotionStatus::m_CurrentJoints[31].m_Pgain);

	param1[2]  = 34;
	param1[3]  = (MotionStatus::m_CurrentJoints[33].m_Pgain);

	param1[4]  = 36;
	param1[5]  = (MotionStatus::m_CurrentJoints[35].m_Pgain);


	manager->m_DXLComm1->GetDXLInstance()->SyncWrite(28, 1, param1, 6);



	param1[0]  = 32;
	param1[1]  = DXL_LOBYTE(MotionStatus::m_CurrentJoints[31].m_Value);
	param1[2]  = DXL_HIBYTE(MotionStatus::m_CurrentJoints[31].m_Value);

	param1[3]  = 34;
	param1[4]  = DXL_LOBYTE(MotionStatus::m_CurrentJoints[33].m_Value);
	param1[5]  = DXL_HIBYTE(MotionStatus::m_CurrentJoints[33].m_Value);

	param1[6]  = 36;
	param1[7]  = DXL_LOBYTE(MotionStatus::m_CurrentJoints[35].m_Value);
	param1[8]  = DXL_HIBYTE(MotionStatus::m_CurrentJoints[35].m_Value);

	manager->m_DXLComm1->GetDXLInstance()->SyncWrite(30, 2, param1, 9);

//	result1 = manager->m_BulkRead1->SendTxPacket();
//
//	static int current_idx = 0;
//	static int watt_idx = 0;


//	if(result1 != COMM_RXSUCCESS)
//	{
//		//printf("result : %d   num : %d\n", result0, debug_num);
//		//debug_num = 0;
//		return &result1;
//	}
//
//	current_idx += 1;
//	watt_idx += 1;
//
//	if(current_idx == CurrentWindowSize)
//		current_idx = 0;
//
//	if(watt_idx == WattWindowSize)
//		watt_idx = 0;
//
//	for(unsigned int idx = 0; idx < bChannel1IDlist.size() ; idx++)
//	{
//		manager->m_BulkRead1->GetDwordValue(bChannel1IDlist[idx], PRO54::P_INDIRECT_DATA_0, (long*)&present_position1);
//		manager->m_BulkRead1->GetWordValue(bChannel1IDlist[idx], PRO54::P_INDIRECT_DATA_4, &present_current1);
//		manager->m_BulkRead1->GetDwordValue(bChannel1IDlist[idx], PRO54::P_INDIRECT_DATA_6, (long*)&present_watt1);
//
//		wCurrentMeanValueArray[bChannel1IDlist[idx] - 1] = (double)(wCurrentMeanValueArray[bChannel1IDlist[idx] - 1]*CurrentWindowSize - wCurrentArray[bChannel1IDlist[idx] - 1][current_idx] + present_current1 )/(double)CurrentWindowSize;
//		wCurrentArray[bChannel1IDlist[idx] - 1][current_idx] = present_current1;
//
//		dwWattMeanValueArray[bChannel1IDlist[idx] - 1] = (double)(dwWattMeanValueArray[bChannel1IDlist[idx] - 1]*WattWindowSize - dwWattArray[bChannel1IDlist[idx] - 1][watt_idx] + present_watt1 )/(double)WattWindowSize;
//		dwWattArray[bChannel1IDlist[idx] - 1][watt_idx] = present_watt1;
//	}
//
//
//	manager->m_BulkRead1->GetWordValue(11, PRO42::P_INDIRECT_DATA_10, &(wRightArmFTVoltageOutput[0]) );
//	manager->m_BulkRead1->GetWordValue(11, PRO42::P_INDIRECT_DATA_12, &(wRightArmFTVoltageOutput[1]) );
//	manager->m_BulkRead1->GetWordValue(11, PRO42::P_INDIRECT_DATA_14, &(wRightArmFTVoltageOutput[2]) );
//	manager->m_BulkRead1->GetWordValue(11, PRO42::P_INDIRECT_DATA_16, &(wRightArmFTVoltageOutput[3]) );
//
//	manager->m_BulkRead1->GetWordValue( 9, PRO42::P_INDIRECT_DATA_10, &(wRightArmFTVoltageOutput[4]) );
//	manager->m_BulkRead1->GetWordValue( 9, PRO42::P_INDIRECT_DATA_12, &(wRightArmFTVoltageOutput[5]) );
//
//	manager->RightArmFTSensor.setCurrentVoltageOutPut(wRightArmFTVoltageOutput[0]*3.3/4095.0,
//													  wRightArmFTVoltageOutput[1]*3.3/4095.0,
//													  wRightArmFTVoltageOutput[2]*3.3/4095.0,
//													  wRightArmFTVoltageOutput[3]*3.3/4095.0,
//													  wRightArmFTVoltageOutput[4]*3.3/4095.0,
//													  wRightArmFTVoltageOutput[5]*3.3/4095.0 );
//
//	manager->RightArmFTSensor.getForceTorque( &(MotionStatus::R_ARM_FX),
//											  &(MotionStatus::R_ARM_FY),
//											  &(MotionStatus::R_ARM_FZ),
//											  &(MotionStatus::R_ARM_TX),
//											  &(MotionStatus::R_ARM_TY),
//											  &(MotionStatus::R_ARM_TZ));
}

//static int debug_num2 = 0;
void* MotionManager::DXLCommProc2(void *param)
{
	//debug_num2 += 1;
	MotionManager* manager = (MotionManager*)param;
	if(manager->comm2)
		return 0;
	manager->m_DXLComm2->SendTxpacket();
	manager->m_DXLComm2->ClearTxpacket();
//	manager->m_BulkRead2->SendTxPacket();
//
//	manager->m_BulkRead2->SendTxPacket();
//	manager->m_BulkRead2->GetWordValue(25, PRO54::P_EXTERNAL_PORT_DATA_1_L, &(wRightLegFTVoltageOutput[0]) );
//	manager->m_BulkRead2->GetWordValue(25, PRO54::P_EXTERNAL_PORT_DATA_2_L, &(wRightLegFTVoltageOutput[1]) );
//	manager->m_BulkRead2->GetWordValue(25, PRO54::P_EXTERNAL_PORT_DATA_3_L, &(wRightLegFTVoltageOutput[2]) );
//	manager->m_BulkRead2->GetWordValue(25, PRO54::P_EXTERNAL_PORT_DATA_4_L, &(wRightLegFTVoltageOutput[3]) );
//
//	manager->m_BulkRead2->GetWordValue(23, PRO54::P_EXTERNAL_PORT_DATA_1_L, &(wRightLegFTVoltageOutput[4]) );
//	manager->m_BulkRead2->GetWordValue(23, PRO54::P_EXTERNAL_PORT_DATA_2_L, &(wRightLegFTVoltageOutput[5]) );
//
//
//	manager->RightLegFTSensor.setCurrentVoltageOutPut((double)wRightLegFTVoltageOutput[0]*3.3/4095.0,
//			(double) wRightLegFTVoltageOutput[1]*3.3/4095.0,
//			(double) wRightLegFTVoltageOutput[2]*3.3/4095.0,
//			(double) wRightLegFTVoltageOutput[3]*3.3/4095.0,
//			(double) wRightLegFTVoltageOutput[4]*3.3/4095.0,
//			(double) wRightLegFTVoltageOutput[5]*3.3/4095.0 );
//
//	manager->RightLegFTSensor.getForceTorque( &(MotionStatus::R_LEG_FX),
//			&(MotionStatus::R_LEG_FY),
//			&(MotionStatus::R_LEG_FZ),
//			&(MotionStatus::R_LEG_TX),
//			&(MotionStatus::R_LEG_TY),
//			&(MotionStatus::R_LEG_TZ));


//	fprintf(fpRLFT, "%d\t%d\t%d\t%d\t%d\t%d\n",
//			wRightLegFTVoltageOutput[0],
//			wRightLegFTVoltageOutput[1],
//			wRightLegFTVoltageOutput[2],
//			wRightLegFTVoltageOutput[3],
//			wRightLegFTVoltageOutput[4],
//			wRightLegFTVoltageOutput[5]);
//
//	static int current_idx = 0;
//	static int watt_idx = 0;
//
//
//	if(result2 != COMM_RXSUCCESS)
//	{
//		//printf("result : %d   num : %d\n", result0, debug_num);
//		//debug_num = 0;
//		return &result2;
//	}
//
//	current_idx += 1;
//	watt_idx += 1;
//
//	if(current_idx == CurrentWindowSize)
//		current_idx = 0;
//
//	if(watt_idx == WattWindowSize)
//		watt_idx = 0;
//
//	for(unsigned int idx = 0; idx < bChannel2IDlist.size() ; idx++)
//	{
//		manager->m_BulkRead2->GetDwordValue(bChannel2IDlist[idx], PRO54::P_INDIRECT_DATA_0, (long*)&present_position2);
//		manager->m_BulkRead2->GetWordValue(bChannel2IDlist[idx], PRO54::P_INDIRECT_DATA_4, &present_current2);
//		manager->m_BulkRead2->GetDwordValue(bChannel2IDlist[idx], PRO54::P_INDIRECT_DATA_6, (long*)&present_watt2);
//
//		wCurrentMeanValueArray[bChannel2IDlist[idx] -1] = (double)(wCurrentMeanValueArray[bChannel2IDlist[idx] -1]*CurrentWindowSize - wCurrentArray[bChannel2IDlist[idx] -1][current_idx] + present_current2 )/(double)CurrentWindowSize;
//		wCurrentArray[bChannel2IDlist[idx] -1][current_idx] = present_current2;
//
//		dwWattMeanValueArray[bChannel2IDlist[idx] -1] = (double)(dwWattMeanValueArray[bChannel2IDlist[idx] -1]*WattWindowSize - dwWattArray[bChannel2IDlist[idx] -1][watt_idx] + present_watt2 )/(double)WattWindowSize;
//		dwWattArray[bChannel2IDlist[idx] -1][watt_idx] = present_watt2;
//	}
//
//	static int FSR1, FSR2,FSR3,FSR4;

//	manager->m_BulkRead2->GetWordValue(23, PRO54::P_INDIRECT_DATA_10, &(wRightLegFTVoltageOutput[0]) );
//	manager->m_BulkRead2->GetWordValue(23, PRO54::P_INDIRECT_DATA_12, &(wRightLegFTVoltageOutput[1]) );
//	manager->m_BulkRead2->GetWordValue(23, PRO54::P_INDIRECT_DATA_14, &(wRightLegFTVoltageOutput[2]) );
//	manager->m_BulkRead2->GetWordValue(23, PRO54::P_INDIRECT_DATA_16, &(wRightLegFTVoltageOutput[3]) );
//
//	manager->m_BulkRead2->GetWordValue(21, PRO54::P_INDIRECT_DATA_10, &(wRightLegFTVoltageOutput[4]) );
//	manager->m_BulkRead2->GetWordValue(21, PRO54::P_INDIRECT_DATA_12, &(wRightLegFTVoltageOutput[5]) );
//	manager->m_BulkRead2->GetDwordValue(15, PRO54::P_PRESENT_POSITION_LL, (long*) &(present_pos_15));
//	manager->m_BulkRead2->GetWordValue(23, PRO54::P_EXTERNAL_PORT_DATA_1_L, &(wRightLegFTVoltageOutput[0]) );
//	manager->m_BulkRead2->GetWordValue(23, PRO54::P_EXTERNAL_PORT_DATA_2_L, &(wRightLegFTVoltageOutput[1]) );
//	manager->m_BulkRead2->GetWordValue(23, PRO54::P_EXTERNAL_PORT_DATA_3_L, &(wRightLegFTVoltageOutput[2]) );
//	manager->m_BulkRead2->GetWordValue(23, PRO54::P_EXTERNAL_PORT_DATA_4_L, &(wRightLegFTVoltageOutput[3]) );
//
//	manager->m_BulkRead2->GetWordValue(21, PRO54::P_EXTERNAL_PORT_DATA_1_L, &(wRightLegFTVoltageOutput[4]) );
//	manager->m_BulkRead2->GetWordValue(21, PRO54::P_EXTERNAL_PORT_DATA_2_L, &(wRightLegFTVoltageOutput[5]) );
//
//	manager->m_BulkRead2->GetWordValue(19, PRO54::P_EXTERNAL_PORT_DATA_1_L, &(FSR1) );
//	manager->m_BulkRead2->GetWordValue(19, PRO54::P_EXTERNAL_PORT_DATA_2_L, &(FSR2) );
//	manager->m_BulkRead2->GetWordValue(19, PRO54::P_EXTERNAL_PORT_DATA_3_L, &(FSR3) );
//	manager->m_BulkRead2->GetWordValue(19, PRO54::P_EXTERNAL_PORT_DATA_4_L, &(FSR4) );
//
//	MotionStatus::R_LEG_FSR1 = FSR1;
//	MotionStatus::R_LEG_FSR2 = FSR2;
//	MotionStatus::R_LEG_FSR3 = FSR3;
//	MotionStatus::R_LEG_FSR4 = FSR4;
//
//	manager->RightLegFTSensor.setCurrentVoltageOutPut((double)wRightLegFTVoltageOutput[0]*3.3/4095.0,
//			(double) wRightLegFTVoltageOutput[1]*3.3/4095.0,
//			(double) wRightLegFTVoltageOutput[2]*3.3/4095.0,
//			(double) wRightLegFTVoltageOutput[3]*3.3/4095.0,
//			(double) wRightLegFTVoltageOutput[4]*3.3/4095.0,
//			(double) wRightLegFTVoltageOutput[5]*3.3/4095.0 );
//
//	manager->RightLegFTSensor.getForceTorque( &(MotionStatus::R_LEG_FX),
//											  &(MotionStatus::R_LEG_FY),
//											  &(MotionStatus::R_LEG_FZ),
//											  &(MotionStatus::R_LEG_TX),
//											  &(MotionStatus::R_LEG_TY),
//											  &(MotionStatus::R_LEG_TZ));
}

unsigned char param3[9];
void* MotionManager::DXLCommProc3(void *param)
{
	MotionManager* manager = (MotionManager*)param;
	if(manager->comm3)
		return 0;
	manager->m_DXLComm3->SendTxpacket();
	manager->m_DXLComm3->ClearTxpacket();

//	static struct timespec StartTime, EndTime;
//	clock_gettime(CLOCK_REALTIME , &StartTime);
//
//	manager->m_BulkRead3->SendTxPacket();
//	manager->m_BulkRead3->GetWordValue(26, PRO54::P_EXTERNAL_PORT_DATA_1_L, &(wLeftLegFTVoltageOutput[0]) );
//	manager->m_BulkRead3->GetWordValue(26, PRO54::P_EXTERNAL_PORT_DATA_2_L, &(wLeftLegFTVoltageOutput[1]) );
//	manager->m_BulkRead3->GetWordValue(26, PRO54::P_EXTERNAL_PORT_DATA_3_L, &(wLeftLegFTVoltageOutput[2]) );
//	manager->m_BulkRead3->GetWordValue(26, PRO54::P_EXTERNAL_PORT_DATA_4_L, &(wLeftLegFTVoltageOutput[3]) );
//
//	manager->m_BulkRead3->GetWordValue(24, PRO54::P_EXTERNAL_PORT_DATA_1_L, &(wLeftLegFTVoltageOutput[4]) );
//	manager->m_BulkRead3->GetWordValue(24, PRO54::P_EXTERNAL_PORT_DATA_2_L, &(wLeftLegFTVoltageOutput[5]) );
//
////	fprintf(fpLLFT, "%d\t%d\t%d\t%d\t%d\t%d\n",
////			wLeftLegFTVoltageOutput[0],
////			wLeftLegFTVoltageOutput[1],
////			wLeftLegFTVoltageOutput[2],
////			wLeftLegFTVoltageOutput[3],
////			wLeftLegFTVoltageOutput[4],
////            wLeftLegFTVoltageOutput[5]);
//
//	manager->LeftLegFTSensor.setCurrentVoltageOutPut(wLeftLegFTVoltageOutput[0]*3.3/4095.0,
//			wLeftLegFTVoltageOutput[1]*3.3/4095.0,
//			wLeftLegFTVoltageOutput[2]*3.3/4095.0,
//			wLeftLegFTVoltageOutput[3]*3.3/4095.0,
//			wLeftLegFTVoltageOutput[4]*3.3/4095.0,
//			wLeftLegFTVoltageOutput[5]*3.3/4095.0 );
//
//	//	printf("%d %d %d %d %d %d\n", wLeftArmFTVoltageOutput[0],
//	//			wLeftArmFTVoltageOutput[1],
//	//			wLeftArmFTVoltageOutput[2],
//	//			wLeftArmFTVoltageOutput[3],
//	//			wLeftArmFTVoltageOutput[4],
//	//			wLeftArmFTVoltageOutput[5]);
//
//	manager->LeftLegFTSensor.getForceTorque( &(MotionStatus::L_LEG_FX),
//			&(MotionStatus::L_LEG_FY),
//			&(MotionStatus::L_LEG_FZ),
//			&(MotionStatus::L_LEG_TX),
//			&(MotionStatus::L_LEG_TY),
//			&(MotionStatus::L_LEG_TZ));

//	manager->m_BulkRead3->SendTxPacket();
//	manager->m_BulkRead3->GetWordValue(26, PRO54::P_EXTERNAL_PORT_DATA_1_L, &(wLeftLegFTVoltageOutput[0]) );
//	manager->m_BulkRead3->GetWordValue(26, PRO54::P_EXTERNAL_PORT_DATA_2_L, &(wLeftLegFTVoltageOutput[1]) );
//	manager->m_BulkRead3->GetWordValue(26, PRO54::P_EXTERNAL_PORT_DATA_3_L, &(wLeftLegFTVoltageOutput[2]) );
//	manager->m_BulkRead3->GetWordValue(26, PRO54::P_EXTERNAL_PORT_DATA_4_L, &(wLeftLegFTVoltageOutput[3]) );
//
//	manager->m_BulkRead3->GetWordValue(24, PRO54::P_EXTERNAL_PORT_DATA_1_L, &(wLeftLegFTVoltageOutput[4]) );
//	manager->m_BulkRead3->GetWordValue(24, PRO54::P_EXTERNAL_PORT_DATA_2_L, &(wLeftLegFTVoltageOutput[5]) );

//	fprintf(fpLLFT, "%d\t%d\t%d\t%d\t%d\t%d\n",
//			wLeftLegFTVoltageOutput[0],
//			wLeftLegFTVoltageOutput[1],
//			wLeftLegFTVoltageOutput[2],
//			wLeftLegFTVoltageOutput[3],
//			wLeftLegFTVoltageOutput[4],
//            wLeftLegFTVoltageOutput[5]);

//	manager->LeftLegFTSensor.setCurrentVoltageOutPut(wLeftLegFTVoltageOutput[0]*3.3/4095.0,
//			wLeftLegFTVoltageOutput[1]*3.3/4095.0,
//			wLeftLegFTVoltageOutput[2]*3.3/4095.0,
//			wLeftLegFTVoltageOutput[3]*3.3/4095.0,
//			wLeftLegFTVoltageOutput[4]*3.3/4095.0,
//			wLeftLegFTVoltageOutput[5]*3.3/4095.0 );

	//	printf("%d %d %d %d %d %d\n", wLeftArmFTVoltageOutput[0],
	//			wLeftArmFTVoltageOutput[1],
	//			wLeftArmFTVoltageOutput[2],
	//			wLeftArmFTVoltageOutput[3],
	//			wLeftArmFTVoltageOutput[4],
	//			wLeftArmFTVoltageOutput[5]);

//	manager->LeftLegFTSensor.getForceTorque( &(MotionStatus::L_LEG_FX),
//			&(MotionStatus::L_LEG_FY),
//			&(MotionStatus::L_LEG_FZ),
//			&(MotionStatus::L_LEG_TX),
//			&(MotionStatus::L_LEG_TY),
//			&(MotionStatus::L_LEG_TZ));

//	static int current_idx = 0;
//	static int watt_idx = 0;
//
//
//	if(result3 != COMM_RXSUCCESS)
//	{
//		//printf("result : %d   num : %d\n", result0, debug_num);
//		//debug_num = 0;
//		return &result3;
//	}
//
//	current_idx += 1;
//	watt_idx += 1;
//
//	if(current_idx == CurrentWindowSize)
//		current_idx = 0;
//
//	if(watt_idx == WattWindowSize)
//		watt_idx = 0;
//
//	for(unsigned int idx = 0; idx < bChannel3IDlist.size() ; idx++)
//	{
//		manager->m_BulkRead3->GetDwordValue(bChannel3IDlist[idx], PRO54::P_INDIRECT_DATA_0, (long*)&present_position3);
//		manager->m_BulkRead3->GetWordValue(bChannel3IDlist[idx], PRO54::P_INDIRECT_DATA_4, &present_current3);
//		manager->m_BulkRead3->GetDwordValue(bChannel3IDlist[idx], PRO54::P_INDIRECT_DATA_6, (long*)&present_watt3);
//
//		wCurrentMeanValueArray[bChannel3IDlist[idx] - 1] = (double)(wCurrentMeanValueArray[bChannel3IDlist[idx] - 1]*CurrentWindowSize - wCurrentArray[bChannel3IDlist[idx] - 1][current_idx] + present_current3 )/(double)CurrentWindowSize;
//		wCurrentArray[bChannel3IDlist[idx] - 1][current_idx] = present_current3;
//
//		dwWattMeanValueArray[bChannel3IDlist[idx] - 1] = (double)(dwWattMeanValueArray[bChannel3IDlist[idx] - 1]*WattWindowSize - dwWattArray[bChannel3IDlist[idx] - 1][watt_idx] + present_watt3 )/(double)WattWindowSize;
//		dwWattArray[bChannel3IDlist[idx] - 1][watt_idx] = present_watt3;
//	}

//	manager->m_BulkRead3->GetWordValue(12, PRO42::P_INDIRECT_DATA_10, &(wLeftArmFTVoltageOutput[0]) );
//	manager->m_BulkRead3->GetWordValue(12, PRO42::P_INDIRECT_DATA_12, &(wLeftArmFTVoltageOutput[1]) );
//	manager->m_BulkRead3->GetWordValue(12, PRO42::P_INDIRECT_DATA_14, &(wLeftArmFTVoltageOutput[2]) );
//	manager->m_BulkRead3->GetWordValue(12, PRO42::P_INDIRECT_DATA_16, &(wLeftArmFTVoltageOutput[3]) );
//
//	manager->m_BulkRead3->GetWordValue(10, PRO42::P_INDIRECT_DATA_10, &(wLeftArmFTVoltageOutput[4]) );
//	manager->m_BulkRead3->GetWordValue(10, PRO42::P_INDIRECT_DATA_12, &(wLeftArmFTVoltageOutput[5]) );
//
//	manager->LeftArmFTSensor.setCurrentVoltageOutPut(wLeftArmFTVoltageOutput[0]*3.3/4095.0,
//													 wLeftArmFTVoltageOutput[1]*3.3/4095.0,
//													 wLeftArmFTVoltageOutput[2]*3.3/4095.0,
//													 wLeftArmFTVoltageOutput[3]*3.3/4095.0,
//													 wLeftArmFTVoltageOutput[4]*3.3/4095.0,
//													 wLeftArmFTVoltageOutput[5]*3.3/4095.0 );
//
////	printf("%d %d %d %d %d %d\n", wLeftArmFTVoltageOutput[0],
////			wLeftArmFTVoltageOutput[1],
////			wLeftArmFTVoltageOutput[2],
////			wLeftArmFTVoltageOutput[3],
////			wLeftArmFTVoltageOutput[4],
////			wLeftArmFTVoltageOutput[5]);
//
//	manager->LeftArmFTSensor.getForceTorque( &(MotionStatus::L_ARM_FX),
//											 &(MotionStatus::L_ARM_FY),
//											 &(MotionStatus::L_ARM_FZ),
//											 &(MotionStatus::L_ARM_TX),
//											 &(MotionStatus::L_ARM_TY),
//											 &(MotionStatus::L_ARM_TZ));

//	clock_gettime(CLOCK_REALTIME , &EndTime);
//
//	double time_dif = (EndTime.tv_sec - StartTime.tv_sec)*1000000000 + ((EndTime.tv_nsec - StartTime.tv_nsec) );
//	time_dif = time_dif/ 1000000000;
//   	fprintf(fp, "%f\n", time_dif);
//	printf(" %f\n",time_dif);
}

void MotionManager::StartTimer()
{
    m_DXLComm0->ClearTxpacket();
    m_DXLComm1->ClearTxpacket();
    m_DXLComm2->ClearTxpacket();
    m_DXLComm3->ClearTxpacket();

	int error = 0;
	struct sched_param param;
	pthread_attr_t attr;

	pthread_attr_init(&attr);

	error = pthread_attr_setschedpolicy(&attr, SCHED_RR);
	if(error != 0)
		fprintf(stderr, "[MotionTimer] pthread_attr_setschedpolicy Error = %d \n", error);
	error = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
	if(error != 0)
		fprintf(stderr, "[MotionTimer] pthread_attr_setinheritsched Error = %d \n", error);

	memset(&param, 0, sizeof(param));
	param.sched_priority = 31; // RealTime
	error = pthread_attr_setschedparam(&attr, &param);
	if(error != 0)
		fprintf(stderr, "[MotionTimer] pthread_attr_setschedparam Error = %d \n", error);

	if(pthread_create(&this->Thread_ID, &attr, this->TimerProc, this) != 0)
		exit(-1);

	this->m_IsTimerRunning = true;
}

void MotionManager::StopTimer()
{
	if(this->m_IsTimerRunning == true)
	{
		this->m_IsTimerStop = true;
		// wait for the thread to end
		if(pthread_join(this->Thread_ID, 0) != 0)
			exit(-1);
		this->m_IsTimerStop = false;
		this->m_IsTimerRunning = false;
	}
}

void* MotionManager::TimerProc(void *param)
{
	MotionManager* manager = (MotionManager*)param;
	static struct timespec next_time;
	clock_gettime(CLOCK_MONOTONIC, &next_time);

	while(!manager->m_IsTimerStop)
	{
        next_time.tv_sec += (next_time.tv_nsec + MotionManager::TIME_UNIT * 1000000) / 1000000000;
        next_time.tv_nsec = (next_time.tv_nsec + MotionManager::TIME_UNIT * 1000000) % 1000000000;

        if(manager != NULL)
        {
        	manager->Process();
        }
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, NULL);
	}

	pthread_exit(NULL);
	return 0;
}

