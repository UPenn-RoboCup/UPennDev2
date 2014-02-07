/*
 * motionmanager.h
 *
 *  Created on: 2013. 1. 11.
 *      Author: zerom
 */

#ifndef MOTIONMANAGER_H_
#define MOTIONMANAGER_H_

#include <list>
#include <vector>
#include <time.h>
#include <pthread.h>
#include <fstream>
#include <iostream>
#include "framework/motion/motionmodule.h"
#include "framework/motion/motionstatus.h"
#include "framework/dynamixel.h"
#include "framework/motion/dxl_comm.h"
#include "framework/motion/PRO42.h"
#include "framework/motion/PRO54.h"
#include "framework/motion/MX28.h"
#include "framework/minIni.h"
#include "framework/sensor/forcetorquesensor/forcetorquesensor.h"
#include <unistd.h>
#define OFFSET_SECTION "Offset"
#define INVALID_VALUE   0X40000000

namespace Thor
{

class MotionManager
{
private:
	static MotionManager* UniqueInstance;
	std::list< MotionModule* > m_Modules;

	bool m_ProcessEnable;
	bool m_Enabled;
	int m_FBGyroCenter;
	int m_RLGyroCenter;
	int m_CalibrationStatus;

	bool m_IsRunning;
	bool m_IsThreadRunning;
	bool m_IsLogging;

	std::ofstream m_LogFileStream;

	MotionManager();

	EnableList* m_EnableList;

	FTSensor RightArmFTSensor;
	FTSensor LeftArmFTSensor;
	FTSensor RightLegFTSensor;
	FTSensor LeftLegFTSensor;


public:
	DXLComm* m_DXLComm0;
	DXLComm* m_DXLComm1;
	DXLComm* m_DXLComm2;
	DXLComm* m_DXLComm3;
	BulkRead* m_BulkRead0;
	BulkRead* m_BulkRead1;
	BulkRead* m_BulkRead2;
	BulkRead* m_BulkRead3;

	bool comm0;
	bool comm1;
	bool comm2;
	bool comm3;

	bool DEBUG_PRINT;
    int m_Offset[MotionStatus::MAXIMUM_NUMBER_OF_JOINTS];

	~MotionManager();

	static MotionManager* GetInstance() { return UniqueInstance; }

	//bool Initialize(int baud_num);
	bool Initialize();
	bool Reinitialize();
	bool InitforTest();
    void Process();
	void SetEnable();
	EnableList* GetEnable()	{ return m_EnableList; }
	void AddModule(MotionModule *module);
	void RemoveModule(MotionModule *module);


	void ResetGyroCalibration() { m_CalibrationStatus = 0; m_FBGyroCenter = 512; m_RLGyroCenter = 512; }
	int GetCalibrationStatus() { return m_CalibrationStatus; }
	void StartLogging();
	void StopLogging();
	void TempStartLogging();
	void TempStopLogging();

    void LoadINISettings(minIni* ini);
    void LoadINISettings(minIni* ini, const std::string &section);
    void SaveINISettings(minIni* ini);
    void SaveINISettings(minIni* ini, const std::string &section);

	int WriteByte(int id, int addr, int value, int* error);
	int WriteWord(int id, int addr, int value, int* error);
	int WriteDWord(int id, int addr, long value, int* error);

	int ReadByte(int id, int addr, int* value, int* error);
	int ReadWord(int id, int addr, int* value, int* error);
	int ReadDWord(int id, int addr, long* value, int* error);

	void CheckFTSensorSetting(void);

	//Timer
private:
	static const int TIME_UNIT = 8;
	pthread_t Thread_ID;
	bool m_IsTimerRunning;
	bool m_IsTimerStop;
	pthread_t m_DXLCommThred0;
	pthread_t m_DXLCommThred1;
	pthread_t m_DXLCommThred2;
	pthread_t m_DXLCommThred3;

public:
	void StartTimer();
	void StopTimer();
	bool IsTimerRunning() { return m_IsTimerRunning; }

protected:
	static void *TimerProc(void *param);
	static void *DXLCommProc0(void *param);
	static void *DXLCommProc1(void *param);
	static void *DXLCommProc2(void *param);
	static void *DXLCommProc3(void *param);
};

} /* namespace Thor */
#endif /* MOTIONMANAGER_H_ */
