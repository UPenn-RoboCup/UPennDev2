#ifndef _DXL_MANAGER_CMD_PROCESS_H_
#define _DXL_MANAGER_CMD_PROCESS_H_


#include "framework/Thor.h"


#define PROGRAM_VERSION		"v1.00"
#define SCREEN_COL			80
#define SCREEN_ROW			34

// Position of Column
#define CMD_COL			2
#define GOAL_COL		21
#define OFFSET_COL		31
#define MODVAL_COL		40
#define PRSVAL_COL		50
#define ERRORS_COL		58
#define P_GAIN_COL      67
#define I_GAIN_COL      74
#define D_GAIN_COL      81


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

#define CMD_ROW		31


int _getch();
bool AskSave();


// Move cursor
void GoToCursor(int col, int row);
void MoveUpCursor();
void MoveDownCursor();
void MoveLeftCursor();
void MoveRightCursor();

// Disp & Drawing
void DrawIntro(Thor::MotionManager *manager);
void DrawEnding();
void DrawPage();
void DrawStep(int index);
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
void OnOffCmd(Thor::MotionManager *manager, bool on, int num_param, int *list);
void SaveCmd(minIni *ini);
void GainCmd(Thor::MotionManager *manager, int value, int pid_col);

void ReadStep(Thor::MotionManager *manager);


bool CheckServoStatus(int id, int *status);
bool CheckPresentStatus(int id, int *status);

#endif
