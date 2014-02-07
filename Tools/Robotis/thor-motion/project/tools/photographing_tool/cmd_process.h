#ifndef _DXL_MANAGER_CMD_PROCESS_H_
#define _DXL_MANAGER_CMD_PROCESS_H_


#include "Thor.h"


#define PROGRAM_VERSION	"v1.00"
#define SCREEN_COL		35

// Position of Column
#define CMD_COL			2
#define PARAM_COL		27
#define TORCUE_COL      45

// Position of Row
enum {
	RESERVED,
	TIMER_MODE_ROW,
	WALKING_MODE_ROW,
    X_OFFSET_ROW,
    Y_OFFSET_ROW,
    Z_OFFSET_ROW,
    ROLL_OFFSET_ROW,
    PITCH_OFFSET_ROW,
    YAW_OFFSET_ROW,
    HIP_PITCH_OFFSET_ROW,
    AUTO_BALANCE_ROW,
    PERIOD_TIME_ROW,
    DSP_RATIO_ROW,
    STEP_FORWARDBACK_RATIO_ROW,
    STEP_FORWARDBACK_ROW,
    STEP_RIGHTLEFT_ROW,
    STEP_DIRECTION_ROW,
    TURNING_AIM_ROW,
    FOOT_HEIGHT_ROW,
    SWING_RIGHTLEFT_ROW,
    SWING_TOPDOWN_ROW,
    PELVIS_OFFSET_ROW,
    ARM_SWING_GAIN_ROW,
    BAL_KNEE_GAIN_ROW,
    BAL_ANKLE_PITCH_GAIN_ROW,
    BAL_HIP_ROLL_GAIN_ROW,
    BAL_ANKLE_ROLL_GAIN_ROW,
    P_GAIN_ROW,
    I_GAIN_ROW,
    D_GAIN_ROW,
    CMD_ROW = 30,
    SCREEN_ROW
};

enum {
	ID_1_ROW,
	ID_2_ROW,
	ID_3_ROW,
	ID_4_ROW,
	ID_5_ROW,
	ID_6_ROW,
	ID_7_ROW,
	ID_8_ROW,
	ID_9_ROW,
	ID_10_ROW,
	ID_11_ROW,
	ID_12_ROW,
	ID_13_ROW,
	ID_14_ROW,
	ID_15_ROW,
	ID_16_ROW,
	ID_17_ROW,
	ID_18_ROW,
	ID_19_ROW,
	ID_20_ROW,
	ID_21_ROW,
	ID_22_ROW,
	ID_23_ROW,
	ID_24_ROW,
	ID_25_ROW,
	ID_26_ROW,
	ID_27_ROW,
	ID_28_ROW,
	ID_R_HAND_ROW,
	ID_L_HAND_ROW
//	CMD_ROW = 30,
//	SCREEN_ROW
};

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
void DrawScreen();
void ClearCmd();
void PrintCmd(const char *message);

// Edit value
void IncreaseValue(bool large);
void DecreaseValue(bool large);

// Command process
void BeginCommandMode();
void EndCommandMode();
void HelpCmd();
void SaveCmd();
void MonitorCmd();


#endif
