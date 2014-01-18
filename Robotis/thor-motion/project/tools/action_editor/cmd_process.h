/*
 * cmd_process.h
 *
 *  Created on: 2013. 1. 23.
 *      Author: hjsong
 */

#ifndef CMD_PROCESS_H_
#define CMD_PROCESS_H_
#ifndef _DXL_MANAGER_CMD_PROCESS_H_
#define _DXL_MANAGER_CMD_PROCESS_H_


#include "framework/Thor.h"


#define PROGRAM_VERSION		"v1.00"
#define SCREEN_COL			80
#define SCREEN_ROW			34

// Position of Column
#define CMD_COL			2
#define STP7_COL		19
#define STP0_COL		25
#define STP1_COL		30
#define STP2_COL		35
#define STP3_COL		40
#define STP4_COL		45
#define STP5_COL		50
#define STP6_COL		55
#define CWSLOPE_COL		60
#define CCWSLOPE_COL	61
#define NAME_COL		63
#define ADDR_COL		72
#define PAGENUM_COL		75
#define PAGEPARAM_COL	76

// Position of Row
#define ID_1_ROW	0
#define ID_2_ROW	1
#define ID_3_ROW	2
#define ID_4_ROW	3
#define ID_5_ROW	4
#define ID_6_ROW	5
#define ID_7_ROW	6
#define ID_8_ROW	7
#define ID_9_ROW	8
#define ID_10_ROW	9
#define ID_11_ROW	10
#define ID_12_ROW	11
#define ID_13_ROW	12
#define ID_14_ROW	13
#define ID_15_ROW	14
#define ID_16_ROW	15
#define ID_17_ROW	16
#define ID_18_ROW	17
#define ID_19_ROW	18
#define ID_20_ROW	19
#define ID_21_ROW	20
#define ID_22_ROW	21
#define ID_23_ROW	22
#define ID_24_ROW	23
#define ID_25_ROW	24
#define ID_26_ROW	25
#define ID_27_ROW	26
#define ID_28_ROW	27
#define ID_29_ROW	28
#define ID_30_ROW	29
#define PAUSE_ROW	30
#define SPEED_ROW	31
#define CMD_ROW		33
#define NAME_ROW		0
#define PAGENUM_ROW		1
#define ADDR_ROW		2
#define PLAYCOUNT_ROW	3
#define STEPNUM_ROW		4
#define PLAYSPEED_ROW	5
#define ACCEL_ROW		6
#define NEXT_ROW		7
#define EXIT_ROW		8


int _getch();
bool AskSave();


// Move cursor
void GoToCursor(int col, int row);
void MoveUpCursor();
void MoveDownCursor();
void MoveLeftCursor();
void MoveRightCursor();

// Disp & Drawing
void DrawIntro();
void DrawEnding();
void DrawPage();
void DrawStep(int index);
void DrawName();
void DrawStepLine(bool erase);
void ClearCmd();
void PrintCmd(const char *message);

// Edit value
void UpDownValue(Thor::MotionManager *manager, int offset);
void SetValue(Thor::MotionManager *manager, int value);
int GetValue();
void ToggleTorque(Thor::MotionManager *manager);

// Command process
void BeginCommandMode();
void EndCommandMode();
void HelpCmd();
void NextCmd();
void PrevCmd();
void PageCmd(int index);
void TimeCmd();
void SpeedCmd();
void PlayCmd(Thor::MotionManager* manager);
void ListCmd();
void OnOffCmd(Thor::MotionManager* manager, bool on, int num_param, int *list);
void WriteStepCmd(int index);
void DeleteStepCmd(int index);
void InsertStepCmd(int index);
void MoveStepCmd(int src, int dst);
void CopyCmd(int index);
void NewCmd();
void GoCmd(Thor::MotionManager *manager, int index);
void SaveCmd();
void NameCmd();

#endif




#endif /* CMD_PROCESS_H_ */
