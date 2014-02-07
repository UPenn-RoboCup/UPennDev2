#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <term.h>
#include <ncurses.h>
#include "cmd_process.h"

using namespace Thor;


int Col = PARAM_COL;
int Row = WALKING_MODE_ROW;
int Old_Col;
int Old_Row;
bool bBeginCommandMode = false;
bool bEdited = false;
int indexPage = 1;


int _getch()
{
	struct termios oldt, newt;
	int ch;
	tcgetattr( STDIN_FILENO, &oldt );
	newt = oldt;
	newt.c_lflag &= ~(ICANON | ECHO);
	tcsetattr( STDIN_FILENO, TCSANOW, &newt );
	ch = getchar();
	tcsetattr( STDIN_FILENO, TCSANOW, &oldt );
	return ch;
}

int kbhit(void)
{
	struct termios oldt, newt;
	int ch;
	int oldf;

	tcgetattr(STDIN_FILENO, &oldt);
	newt = oldt;
	newt.c_lflag &= ~(ICANON | ECHO);
	tcsetattr(STDIN_FILENO, TCSANOW, &newt);
	oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
	fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

	ch = getchar();

	tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
	fcntl(STDIN_FILENO, F_SETFL, oldf);

	if(ch != EOF)
	{
		ungetc(ch, stdin);
		return 1;
	}

	return 0;
}

struct termios oldterm, new_term;
void set_stdin(void)
{
	tcgetattr(0,&oldterm);
	new_term = oldterm;
	new_term.c_lflag &= ~(ICANON | ECHO | ISIG); // 의미는 struct termios를 찾으면 됨.
	new_term.c_cc[VMIN] = 1;
	new_term.c_cc[VTIME] = 0;
	tcsetattr(0, TCSANOW, &new_term);
}

void reset_stdin(void)
{
	tcsetattr(0, TCSANOW, &oldterm);
}

bool AskSave()
{
	if(bEdited == true)
	{
		PrintCmd("Are you sure? (y/n)");
		if(_getch() != 'y')
		{
			ClearCmd();
			return true;
		}
	}

	return false;
}


void GoToCursor(int col, int row)
{
	char *cursor;
	char *esc_sequence;
	cursor = tigetstr("cup");
	esc_sequence = tparm(cursor, row, col);
	putp(esc_sequence);

	Col = col;
	Row = row;
}

void MoveUpCursor()
{
	if(Col == PARAM_COL)
	{
		if(Row > 0)
			GoToCursor(Col, Row-1);
	}
	else if(Col == TORCUE_COL)
	{
		if(Row > 0)
			GoToCursor(Col, Row-1);
	}
	else
		return;
}

void MoveDownCursor()
{
	if(Col == PARAM_COL)
	{
		if(Row < CMD_ROW - 1)
			GoToCursor(Col, Row+1);
	}
	else if(Col == TORCUE_COL)
	{
		if(Row < CMD_ROW - 1)
			GoToCursor(Col, Row+1);
	}
	else
		return;
}

void MoveLeftCursor()
{
	if(Col == PARAM_COL)
		return;
	else
		GoToCursor(PARAM_COL, Row);
}

void MoveRightCursor()
{
	if(Col == TORCUE_COL)
		return;
	else
		GoToCursor(TORCUE_COL, Row);
}

void DrawIntro(MotionManager *manager)
{	
	int n = 0;
	unsigned char *param = new unsigned char[MotionStatus::m_CurrentJoints.size()*9 ];
	int wGoalPosition, wStartPosition, wDistance;

	for(unsigned int jointIndex = 0; jointIndex < MotionStatus::m_CurrentJoints.size(); jointIndex++)
	{
		int id = -1;
		id = MotionStatus::m_CurrentJoints[jointIndex].m_ID;
		wStartPosition = MotionStatus::m_CurrentJoints[jointIndex].m_Value;

		if(id == 25 || id == 26 || id == 27 || id  == 28)
		{
			MotionStatus::m_CurrentJoints[jointIndex].m_Value = 0;
			//Walking::GetInstance()->m_RobotInfo[jointIndex].m_Value = 0;
		}
	}

	Walking::GetInstance()->Initialize();

	for(unsigned int jointIndex = 0; jointIndex < MotionStatus::m_CurrentJoints.size(); jointIndex++)
	{
		int id = -1;
		if(id >= 1 || id <= 28)
		{
			id = MotionStatus::m_CurrentJoints[jointIndex].m_ID;
			wStartPosition = MotionStatus::m_CurrentJoints[jointIndex].m_Value;
			wGoalPosition = Walking::GetInstance()->m_RobotInfo[jointIndex].m_Value;
			//fprintf(stderr, "Goal Pos : %d,  %d\n",id, wGoalPosition);

			wDistance = 500;


			param[n++] = (unsigned char)id;
			param[n++] = DXL_LOBYTE(DXL_LOWORD(wGoalPosition));
			param[n++] = DXL_HIBYTE(DXL_LOWORD(wGoalPosition));
			param[n++] = DXL_LOBYTE(DXL_HIWORD(wGoalPosition));
			param[n++] = DXL_HIBYTE(DXL_HIWORD(wGoalPosition));

			param[n++] = DXL_LOBYTE(DXL_LOWORD(wDistance));
			param[n++] = DXL_HIBYTE(DXL_LOWORD(wDistance));
			param[n++] = DXL_LOBYTE(DXL_HIWORD(wDistance));
			param[n++] = DXL_HIBYTE(DXL_HIWORD(wDistance));
		}
	}

	manager->m_DXLComm0->GetDXLInstance()->SyncWrite(PRO54::P_GOAL_POSITION_LL, 8, param, MotionStatus::m_CurrentJoints.size()*9 );
	manager->m_DXLComm1->GetDXLInstance()->SyncWrite(PRO54::P_GOAL_POSITION_LL, 8, param, MotionStatus::m_CurrentJoints.size()*9 );
	manager->m_DXLComm2->GetDXLInstance()->SyncWrite(PRO54::P_GOAL_POSITION_LL, 8, param, MotionStatus::m_CurrentJoints.size()*9 );
	manager->m_DXLComm3->GetDXLInstance()->SyncWrite(PRO54::P_GOAL_POSITION_LL, 8, param, MotionStatus::m_CurrentJoints.size()*9 );

	sleep(10);
	n=0;
	for(unsigned int jointIndex = 0; jointIndex < MotionStatus::m_CurrentJoints.size(); jointIndex++)
	{
		int id = -1;
		id = MotionStatus::m_CurrentJoints[jointIndex].m_ID;
		if(id >= 1 || id <= 28)
		{
			wStartPosition = MotionStatus::m_CurrentJoints[jointIndex].m_Value;
			wGoalPosition = Walking::GetInstance()->m_RobotInfo[jointIndex].m_Value;
			if( wStartPosition > wGoalPosition )
				wDistance = wStartPosition - wGoalPosition;
			else
				wDistance = wGoalPosition - wStartPosition;

			wDistance =17000;

			param[n++] = (unsigned char)id;
			param[n++] = DXL_LOBYTE(DXL_LOWORD(wGoalPosition));
			param[n++] = DXL_HIBYTE(DXL_LOWORD(wGoalPosition));
			param[n++] = DXL_LOBYTE(DXL_HIWORD(wGoalPosition));
			param[n++] = DXL_HIBYTE(DXL_HIWORD(wGoalPosition));

			param[n++] = DXL_LOBYTE(DXL_LOWORD(wDistance));
			param[n++] = DXL_HIBYTE(DXL_LOWORD(wDistance));
			param[n++] = DXL_LOBYTE(DXL_HIWORD(wDistance));
			param[n++] = DXL_HIBYTE(DXL_HIWORD(wDistance));
		}
	}

	manager->m_DXLComm0->GetDXLInstance()->SyncWrite(PRO54::P_GOAL_POSITION_LL, 8, param, MotionStatus::m_CurrentJoints.size()*9 );
	manager->m_DXLComm1->GetDXLInstance()->SyncWrite(PRO54::P_GOAL_POSITION_LL, 8, param, MotionStatus::m_CurrentJoints.size()*9 );
	manager->m_DXLComm2->GetDXLInstance()->SyncWrite(PRO54::P_GOAL_POSITION_LL, 8, param, MotionStatus::m_CurrentJoints.size()*9 );
	manager->m_DXLComm3->GetDXLInstance()->SyncWrite(PRO54::P_GOAL_POSITION_LL, 8, param, MotionStatus::m_CurrentJoints.size()*9 );

	delete[] param;

	int nrows, ncolumns;
	setupterm(NULL, fileno(stdout), (int *)0);
	nrows = tigetnum("lines");
	ncolumns = tigetnum("cols");

	system("clear");
	printf("\n");
	printf("[Walking Tuner for Thor %s]\n", PROGRAM_VERSION);
	printf("\n");
	printf(" *Terminal screen size must be %d(col)x%d(row).\n", SCREEN_COL, SCREEN_ROW);
	printf(" *Current terminal has %d columns and %d rows.\n", ncolumns, nrows);
	printf("\n");
	printf("\n");
	printf("Press any key to start program...\n");
	_getch();

	DrawScreen();
}

void DrawEnding()
{
	system("clear");
	printf("\n");
	printf("Terminate Walking Tuner");
	printf("\n");
}

void DrawScreen()
{
	int old_col = Col;
	int old_row = Row;

	system("clear");
	GoToCursor(0, 0);

	// Display menu
	//      01234567890123456789012345678901234567890123456789  Total:46x30
	printf("BLANK                              |[ID_01]    \n"); // 0
	printf("Timer Mode(on/off)                 |[ID_02]    \n"); // 1
	printf("Walking Mode(on/off)               |[ID_03]    \n"); // 2
	printf("X offset(mm)                       |[ID_04]    \n"); // 3
	printf("Y offset(mm)                       |[ID_05]    \n"); // 4
	printf("Z offset(mm)                       |[ID_06]    \n"); // 5
	printf("Roll(x) offset(degree)             |[ID_07]    \n"); // 6
	printf("Pitch(y) offset(degree)            |[ID_08]    \n"); // 7
	printf("Yaw(z) offset(degree)              |[ID_09]    \n"); // 8
	printf("Hip pitch offset(degree)           |[ID_10]    \n"); // 9
	printf("Auto balance(on/off)               |[ID_11]    \n"); // 0
	printf("Period time(msec)                  |[ID_12]    \n"); // 1
	printf("DSP ratio                          |[ID_13]    \n"); // 2
	printf("Step forward/back ratio            |[ID_14]    \n"); // 3
	printf("Step forward/back(mm)              |[ID_15]    \n"); // 4
	printf("Step right/left(mm)                |[ID_16]    \n"); // 5
	printf("Step direction(degree)             |[ID_17]    \n"); // 6
	printf("Turning aim(on/off)                |[ID_18]    \n"); // 7
	printf("Foot height(mm)                    |[ID_19]    \n"); // 8
	printf("Swing right/left(mm)               |[ID_20]    \n"); // 9
	printf("Swing top/down(mm)                 |[ID_21]    \n"); // 0
	printf("Pelvis offset(degree)              |[ID_22]    \n"); // 1
	printf("Arm swing gain                     |[ID_23]    \n"); // 2
	printf("Balance knee gain                  |[ID_24]    \n"); // 3
	printf("Balance ankle pitch gain           |[ID_25]    \n"); // 4
	printf("Balance hip roll gain              |[ID_26]    \n"); // 5
	printf("Balance ankle roll gain            |[ID_27]    \n"); // 6
	printf("P gain                             |[ID_28]    \n"); // 7
	printf("I gain                             |[RHAND]    \n"); // 8
	printf("D gain                             |[LHAND]    \n"); // 9
	ClearCmd(); //0

	GoToCursor(PARAM_COL, TIMER_MODE_ROW);
	if(MotionManager::GetInstance()->IsTimerRunning() == true)
		printf("ON     ");
	else
		printf("OFF    ");

	GoToCursor(PARAM_COL, WALKING_MODE_ROW);
	if(Walking::GetInstance()->IsRunning() == true)
		printf("ON     ");
	else
		printf("OFF    ");

	GoToCursor(PARAM_COL, X_OFFSET_ROW);
	printf("%d    ", (int)Walking::GetInstance()->X_OFFSET);

	GoToCursor(PARAM_COL, Y_OFFSET_ROW);
	printf("%d    ", (int)Walking::GetInstance()->Y_OFFSET);

	GoToCursor(PARAM_COL, Z_OFFSET_ROW);
	printf("%d    ", (int)Walking::GetInstance()->Z_OFFSET);

	GoToCursor(PARAM_COL, ROLL_OFFSET_ROW);
	printf("%.1f    ", Walking::GetInstance()->R_OFFSET);

	GoToCursor(PARAM_COL, PITCH_OFFSET_ROW);
	printf("%.1f    ", Walking::GetInstance()->P_OFFSET);

	GoToCursor(PARAM_COL, YAW_OFFSET_ROW);
	printf("%.1f    ", Walking::GetInstance()->A_OFFSET);

	GoToCursor(PARAM_COL, HIP_PITCH_OFFSET_ROW);
	printf("%.1f    ", Walking::GetInstance()->HIP_PITCH_OFFSET);

	GoToCursor(PARAM_COL, AUTO_BALANCE_ROW);
	if(Walking::GetInstance()->BALANCE_ENABLE == true)
		printf("ON     ");
	else
		printf("OFF    ");

	GoToCursor(PARAM_COL, PERIOD_TIME_ROW);
	printf("%d    ", (int)Walking::GetInstance()->PERIOD_TIME);

	GoToCursor(PARAM_COL, DSP_RATIO_ROW);
	printf("%.2f    ", Walking::GetInstance()->DSP_RATIO);

	GoToCursor(PARAM_COL, STEP_FORWARDBACK_RATIO_ROW);
	printf("%.2f    ", Walking::GetInstance()->STEP_FB_RATIO);

	GoToCursor(PARAM_COL, STEP_FORWARDBACK_ROW);
	printf("%d    ", (int)Walking::GetInstance()->X_MOVE_AMPLITUDE);

	GoToCursor(PARAM_COL, STEP_RIGHTLEFT_ROW);
	printf("%d    ", (int)Walking::GetInstance()->Y_MOVE_AMPLITUDE);

	GoToCursor(PARAM_COL, STEP_DIRECTION_ROW);
	printf("%d    ", (int)Walking::GetInstance()->A_MOVE_AMPLITUDE);

	GoToCursor(PARAM_COL, TURNING_AIM_ROW);
	if(Walking::GetInstance()->A_MOVE_AIM_ON == true)
		printf("ON     ");
	else
		printf("OFF    ");

	GoToCursor(PARAM_COL, FOOT_HEIGHT_ROW);
	printf("%d    ", (int)Walking::GetInstance()->Z_MOVE_AMPLITUDE);

	GoToCursor(PARAM_COL, SWING_RIGHTLEFT_ROW);
	printf("%.1f    ", Walking::GetInstance()->Y_SWAP_AMPLITUDE);

	GoToCursor(PARAM_COL, SWING_TOPDOWN_ROW);
	printf("%d    ", (int)Walking::GetInstance()->Z_SWAP_AMPLITUDE);

	GoToCursor(PARAM_COL, PELVIS_OFFSET_ROW);
	printf("%.1f    ", Walking::GetInstance()->PELVIS_OFFSET);

	GoToCursor(PARAM_COL, ARM_SWING_GAIN_ROW);
	printf("%.1f    ", Walking::GetInstance()->ARM_SWING_GAIN);

	GoToCursor(PARAM_COL, BAL_KNEE_GAIN_ROW);
	printf("%.2f    ", Walking::GetInstance()->BALANCE_KNEE_GAIN);

	GoToCursor(PARAM_COL, BAL_ANKLE_PITCH_GAIN_ROW);
	printf("%.2f    ", Walking::GetInstance()->BALANCE_ANKLE_PITCH_GAIN);

	GoToCursor(PARAM_COL, BAL_HIP_ROLL_GAIN_ROW);
	printf("%.2f    ", Walking::GetInstance()->BALANCE_HIP_ROLL_GAIN);

	GoToCursor(PARAM_COL, BAL_ANKLE_ROLL_GAIN_ROW);
	printf("%.2f    ", Walking::GetInstance()->BALANCE_ANKLE_ROLL_GAIN);

	GoToCursor(PARAM_COL, P_GAIN_ROW);
	printf("%d    ", Walking::GetInstance()->P_GAIN);

	GoToCursor(PARAM_COL, I_GAIN_ROW);
	printf("%d    ", Walking::GetInstance()->I_GAIN);

	GoToCursor(PARAM_COL, D_GAIN_ROW);
	printf("%d    ", Walking::GetInstance()->D_GAIN);


	for(int i = ID_1_ROW; i < ID_L_HAND_ROW + 1 ; i++)
	{
		GoToCursor(TORCUE_COL, i);
		printf("O");
	}

	GoToCursor(old_col, old_row);
}

void ClearCmd()
{
	PrintCmd("");
}

void PrintCmd(const char *message)
{
	int len = strlen(message);
	GoToCursor(0, CMD_ROW);

	printf( "] %s", message);
	for(int i=0; i<(SCREEN_COL - (len + 2)); i++)
		printf(" ");

	GoToCursor(len + 2, CMD_ROW);
}

void IncreaseValue(bool large)
{
	int col;
	int row;
	if(bBeginCommandMode == true)
	{
		col = Old_Col;
		row = Old_Row;
	}
	else
	{
		col = Col;
		row = Row;
	}

//	if(col != PARAM_COL)
//		return;

	GoToCursor(col, row);

	if(col == PARAM_COL)
	{
		switch(row)
		{
		case TIMER_MODE_ROW:
			MotionManager::GetInstance()->StartTimer();
			printf("ON    ");
			break;
		case WALKING_MODE_ROW:
			MotionManager::GetInstance()->StartLogging();
			MotionManager::GetInstance()->StartTimer();
			Walking::GetInstance()->Start();
			printf("ON    ");
			GoToCursor(PARAM_COL, TIMER_MODE_ROW);
			printf("ON    ");
			break;

		case X_OFFSET_ROW:
			if(large == true)
				Walking::GetInstance()->X_OFFSET += 10;
			else
				Walking::GetInstance()->X_OFFSET += 1;
			printf("%d    ", (int)Walking::GetInstance()->X_OFFSET);
			break;

		case Y_OFFSET_ROW:
			if(large == true)
				Walking::GetInstance()->Y_OFFSET += 10;
			else
				Walking::GetInstance()->Y_OFFSET += 1;
			printf("%d    ", (int)Walking::GetInstance()->Y_OFFSET);
			break;

		case Z_OFFSET_ROW:
			if(large == true)
				Walking::GetInstance()->Z_OFFSET += 10;
			else
				Walking::GetInstance()->Z_OFFSET += 1;
			printf("%d    ", (int)Walking::GetInstance()->Z_OFFSET);
			break;

		case ROLL_OFFSET_ROW:
			if(large == true)
				Walking::GetInstance()->R_OFFSET += 1.0;
			else
				Walking::GetInstance()->R_OFFSET += 0.1;
			printf("%.1f    ", Walking::GetInstance()->R_OFFSET);
			break;

		case PITCH_OFFSET_ROW:
			if(large == true)
				Walking::GetInstance()->P_OFFSET += 1.0;
			else
				Walking::GetInstance()->P_OFFSET += 0.1;
			printf("%.1f    ", Walking::GetInstance()->P_OFFSET);
			break;

		case YAW_OFFSET_ROW:
			if(large == true)
				Walking::GetInstance()->Y_OFFSET += 1.0;
			else
				Walking::GetInstance()->Y_OFFSET += 0.1;
			printf("%.1f    ", Walking::GetInstance()->Y_OFFSET);
			break;

		case HIP_PITCH_OFFSET_ROW:
			if(large == true)
				Walking::GetInstance()->HIP_PITCH_OFFSET += 1.0;
			else
				Walking::GetInstance()->HIP_PITCH_OFFSET += 0.1;
			printf("%.1f    ", Walking::GetInstance()->HIP_PITCH_OFFSET);
			break;

		case AUTO_BALANCE_ROW:
			Walking::GetInstance()->BALANCE_ENABLE = true;
			printf("ON    ");
			break;

		case PERIOD_TIME_ROW:
			if(large == true)
				Walking::GetInstance()->PERIOD_TIME += 10;
			else
				Walking::GetInstance()->PERIOD_TIME += 1;
			printf("%d    ", (int)Walking::GetInstance()->PERIOD_TIME);
			break;

		case DSP_RATIO_ROW:
			if(large == true)
				Walking::GetInstance()->DSP_RATIO += 0.1;
			else
				Walking::GetInstance()->DSP_RATIO += 0.01;
			printf("%.2f    ", Walking::GetInstance()->DSP_RATIO);
			break;

		case STEP_FORWARDBACK_RATIO_ROW:
			if(large == true)
				Walking::GetInstance()->STEP_FB_RATIO += 0.1;
			else
				Walking::GetInstance()->STEP_FB_RATIO += 0.01;
			printf("%.2f    ", Walking::GetInstance()->STEP_FB_RATIO);
			break;

		case STEP_FORWARDBACK_ROW:
			if(large == true)
				Walking::GetInstance()->X_MOVE_AMPLITUDE += 10;
			else
				Walking::GetInstance()->X_MOVE_AMPLITUDE += 1;
			printf("%d    ", (int)Walking::GetInstance()->X_MOVE_AMPLITUDE);
			break;

		case STEP_RIGHTLEFT_ROW:
			if(large == true)
				Walking::GetInstance()->Y_MOVE_AMPLITUDE += 10;
			else
				Walking::GetInstance()->Y_MOVE_AMPLITUDE += 1;
			printf("%d    ", (int)Walking::GetInstance()->Y_MOVE_AMPLITUDE);
			break;

		case STEP_DIRECTION_ROW:
			if(large == true)
				Walking::GetInstance()->A_MOVE_AMPLITUDE += 10;
			else
				Walking::GetInstance()->A_MOVE_AMPLITUDE += 1;
			printf("%d    ", (int)Walking::GetInstance()->A_MOVE_AMPLITUDE);
			break;

		case TURNING_AIM_ROW:
			Walking::GetInstance()->A_MOVE_AIM_ON = true;
			printf("ON   ");
			break;

		case FOOT_HEIGHT_ROW:
			if(large == true)
				Walking::GetInstance()->Z_MOVE_AMPLITUDE += 10;
			else
				Walking::GetInstance()->Z_MOVE_AMPLITUDE += 1;
			printf("%d    ", (int)Walking::GetInstance()->Z_MOVE_AMPLITUDE);
			break;

		case SWING_RIGHTLEFT_ROW:
			if(large == true)
				Walking::GetInstance()->Y_SWAP_AMPLITUDE += 1.0;
			else
				Walking::GetInstance()->Y_SWAP_AMPLITUDE += 0.1;
			printf("%.1f    ", Walking::GetInstance()->Y_SWAP_AMPLITUDE);
			break;

		case SWING_TOPDOWN_ROW:
			if(large == true)
				Walking::GetInstance()->Z_SWAP_AMPLITUDE += 10;
			else
				Walking::GetInstance()->Z_SWAP_AMPLITUDE += 1;
			printf("%d    ", (int)Walking::GetInstance()->Z_SWAP_AMPLITUDE);
			break;

		case PELVIS_OFFSET_ROW:
			if(large == true)
				Walking::GetInstance()->PELVIS_OFFSET += 1.0;
			else
				Walking::GetInstance()->PELVIS_OFFSET += 0.1;
			printf("%.1f    ", Walking::GetInstance()->PELVIS_OFFSET);
			break;

		case ARM_SWING_GAIN_ROW:
			if(large == true)
				Walking::GetInstance()->ARM_SWING_GAIN += 0.10;
			else
				Walking::GetInstance()->ARM_SWING_GAIN += 0.01;
			printf("%.1f    ", Walking::GetInstance()->ARM_SWING_GAIN);
			break;

		case BAL_KNEE_GAIN_ROW:
			if(large == true)
				Walking::GetInstance()->BALANCE_KNEE_GAIN += 0.1;
			else
				Walking::GetInstance()->BALANCE_KNEE_GAIN += 0.01;
			printf("%.2f    ", Walking::GetInstance()->BALANCE_KNEE_GAIN);
			break;

		case BAL_ANKLE_PITCH_GAIN_ROW:
			if(large == true)
				Walking::GetInstance()->BALANCE_ANKLE_PITCH_GAIN += 0.1;
			else
				Walking::GetInstance()->BALANCE_ANKLE_PITCH_GAIN += 0.01;
			printf("%.2f    ", Walking::GetInstance()->BALANCE_ANKLE_PITCH_GAIN);
			break;

		case BAL_HIP_ROLL_GAIN_ROW:
			if(large == true)
				Walking::GetInstance()->BALANCE_HIP_ROLL_GAIN += 0.1;
			else
				Walking::GetInstance()->BALANCE_HIP_ROLL_GAIN += 0.01;
			printf("%.2f    ", Walking::GetInstance()->BALANCE_HIP_ROLL_GAIN);
			break;

		case BAL_ANKLE_ROLL_GAIN_ROW:
			if(large == true)
				Walking::GetInstance()->BALANCE_ANKLE_ROLL_GAIN += 0.1;
			else
				Walking::GetInstance()->BALANCE_ANKLE_ROLL_GAIN += 0.01;
			printf("%.2f    ", Walking::GetInstance()->BALANCE_ANKLE_ROLL_GAIN);
			break;

		case P_GAIN_ROW:
			if(large == true)
				Walking::GetInstance()->P_GAIN += 10;
			else
				Walking::GetInstance()->P_GAIN += 1;
			printf("%d    ", Walking::GetInstance()->P_GAIN);
			break;

		case I_GAIN_ROW:
			if(large == true)
				Walking::GetInstance()->I_GAIN += 10;
			else
				Walking::GetInstance()->I_GAIN += 1;
			printf("%d    ", Walking::GetInstance()->I_GAIN);
			break;

		case D_GAIN_ROW:
			if(large == true)
				Walking::GetInstance()->D_GAIN += 10;
			else
				Walking::GetInstance()->D_GAIN += 1;
			printf("%d    ", Walking::GetInstance()->D_GAIN);
			break;
		}
	}
	else
	{
		printf("O  ");
		if(row == ID_R_HAND_ROW)
		{
			MotionManager::GetInstance()->WriteByte(29, 24, 1, 0);
			MotionManager::GetInstance()->WriteByte(31, 24, 1, 0);
			MotionManager::GetInstance()->WriteByte(33, 24, 1, 0);
		}
		else if(row == ID_L_HAND_ROW)
		{
			MotionManager::GetInstance()->WriteByte(30, 24, 1, 0);
			MotionManager::GetInstance()->WriteByte(32, 24, 1, 0);
			MotionManager::GetInstance()->WriteByte(34, 24, 1, 0);
		}
		else
			MotionManager::GetInstance()->WriteByte(row+1, 562, 1, 0);
	}
	GoToCursor(col, row);
}

void DecreaseValue(bool large)
{
	int col;
	int row;
	if(bBeginCommandMode == true)
	{
		col = Old_Col;
		row = Old_Row;
	}
	else
	{
		col = Col;
		row = Row;
	}

//	if(col != PARAM_COL)
//		return;

	GoToCursor(col, row);

	if(col == PARAM_COL)
	{
		switch(row)
		{
		case TIMER_MODE_ROW:
			MotionManager::GetInstance()->StopTimer();
			printf("OFF");
			break;
		case WALKING_MODE_ROW:
			Walking::GetInstance()->Stop();
			MotionManager::GetInstance()->StopLogging();
			printf("OFF");
			GoToCursor(PARAM_COL, STEP_FORWARDBACK_ROW);
			Walking::GetInstance()->X_MOVE_AMPLITUDE = 0;
			printf("%d    ", (int)Walking::GetInstance()->X_MOVE_AMPLITUDE);
			GoToCursor(PARAM_COL, STEP_RIGHTLEFT_ROW);
			Walking::GetInstance()->Y_MOVE_AMPLITUDE = 0;
			printf("%d    ", (int)Walking::GetInstance()->Y_MOVE_AMPLITUDE);
			GoToCursor(PARAM_COL, STEP_DIRECTION_ROW);
			Walking::GetInstance()->A_MOVE_AMPLITUDE = 0;
			printf("%.1f    ", Walking::GetInstance()->A_MOVE_AMPLITUDE);
			break;

		case X_OFFSET_ROW:
			if(large == true)
				Walking::GetInstance()->X_OFFSET -= 10;
			else
				Walking::GetInstance()->X_OFFSET -= 1;
			printf("%d    ", (int)Walking::GetInstance()->X_OFFSET);
			break;

		case Y_OFFSET_ROW:
			if(large == true)
				Walking::GetInstance()->Y_OFFSET -= 10;
			else
				Walking::GetInstance()->Y_OFFSET -= 1;
			printf("%d    ", (int)Walking::GetInstance()->Y_OFFSET);
			break;

		case Z_OFFSET_ROW:
			if(large == true)
				Walking::GetInstance()->Z_OFFSET -= 10;
			else
				Walking::GetInstance()->Z_OFFSET -= 1;
			printf("%d    ", (int)Walking::GetInstance()->Z_OFFSET);
			break;

		case ROLL_OFFSET_ROW:
			if(large == true)
				Walking::GetInstance()->R_OFFSET -= 1.0;
			else
				Walking::GetInstance()->R_OFFSET -= 0.1;
			printf("%.1f    ", Walking::GetInstance()->R_OFFSET);
			break;

		case PITCH_OFFSET_ROW:
			if(large == true)
				Walking::GetInstance()->P_OFFSET -= 1.0;
			else
				Walking::GetInstance()->P_OFFSET -= 0.1;
			printf("%.1f    ", Walking::GetInstance()->P_OFFSET);
			break;

		case YAW_OFFSET_ROW:
			if(large == true)
				Walking::GetInstance()->Y_OFFSET -= 1.0;
			else
				Walking::GetInstance()->Y_OFFSET -= 0.1;
			printf("%.1f    ", Walking::GetInstance()->Y_OFFSET);
			break;

		case HIP_PITCH_OFFSET_ROW:
			if(large == true)
				Walking::GetInstance()->HIP_PITCH_OFFSET -= 1.0;
			else
				Walking::GetInstance()->HIP_PITCH_OFFSET -= 0.1;
			printf("%.1f    ", Walking::GetInstance()->HIP_PITCH_OFFSET);
			break;

		case AUTO_BALANCE_ROW:
			Walking::GetInstance()->BALANCE_ENABLE = false;
			printf("OFF   ");
			break;

		case PERIOD_TIME_ROW:
			if(large == true)
				Walking::GetInstance()->PERIOD_TIME -= 10;
			else
				Walking::GetInstance()->PERIOD_TIME -= 1;
			printf("%d    ", (int)Walking::GetInstance()->PERIOD_TIME);
			break;

		case DSP_RATIO_ROW:
			if(large == true)
				Walking::GetInstance()->DSP_RATIO -= 0.1;
			else
				Walking::GetInstance()->DSP_RATIO -= 0.01;
			printf("%.2f    ", Walking::GetInstance()->DSP_RATIO);
			break;

		case STEP_FORWARDBACK_RATIO_ROW:
			if(large == true)
				Walking::GetInstance()->STEP_FB_RATIO -= 0.1;
			else
				Walking::GetInstance()->STEP_FB_RATIO -= 0.01;
			printf("%.2f    ", Walking::GetInstance()->STEP_FB_RATIO);
			break;

		case STEP_FORWARDBACK_ROW:
			if(large == true)
				Walking::GetInstance()->X_MOVE_AMPLITUDE -= 10;
			else
				Walking::GetInstance()->X_MOVE_AMPLITUDE -= 1;
			printf("%d    ", (int)Walking::GetInstance()->X_MOVE_AMPLITUDE);
			break;

		case STEP_RIGHTLEFT_ROW:
			if(large == true)
				Walking::GetInstance()->Y_MOVE_AMPLITUDE -= 10;
			else
				Walking::GetInstance()->Y_MOVE_AMPLITUDE -= 1;
			printf("%d    ", (int)Walking::GetInstance()->Y_MOVE_AMPLITUDE);
			break;

		case STEP_DIRECTION_ROW:
			if(large == true)
				Walking::GetInstance()->A_MOVE_AMPLITUDE -= 10;
			else
				Walking::GetInstance()->A_MOVE_AMPLITUDE -= 1;
			printf("%d    ", (int)Walking::GetInstance()->A_MOVE_AMPLITUDE);
			break;

		case TURNING_AIM_ROW:
			Walking::GetInstance()->A_MOVE_AIM_ON = false;
			printf("OFF   ");
			break;

		case FOOT_HEIGHT_ROW:
			if(large == true)
				Walking::GetInstance()->Z_MOVE_AMPLITUDE -= 10;
			else
				Walking::GetInstance()->Z_MOVE_AMPLITUDE -= 1;
			printf("%d    ", (int)Walking::GetInstance()->Z_MOVE_AMPLITUDE);
			break;

		case SWING_RIGHTLEFT_ROW:
			if(large == true)
				Walking::GetInstance()->Y_SWAP_AMPLITUDE -= 1.0;
			else
				Walking::GetInstance()->Y_SWAP_AMPLITUDE -= 0.1;
			printf("%.1f    ", Walking::GetInstance()->Y_SWAP_AMPLITUDE);
			break;

		case SWING_TOPDOWN_ROW:
			if(large == true)
				Walking::GetInstance()->Z_SWAP_AMPLITUDE -= 10;
			else
				Walking::GetInstance()->Z_SWAP_AMPLITUDE -= 1;
			printf("%d    ", (int)Walking::GetInstance()->Z_SWAP_AMPLITUDE);
			break;

		case PELVIS_OFFSET_ROW:
			if(large == true)
				Walking::GetInstance()->PELVIS_OFFSET -= 1.0;
			else
				Walking::GetInstance()->PELVIS_OFFSET -= 0.1;
			printf("%.1f    ", Walking::GetInstance()->PELVIS_OFFSET);
			break;

		case ARM_SWING_GAIN_ROW:
			if(large == true)
				Walking::GetInstance()->ARM_SWING_GAIN -= 1.0;
			else
				Walking::GetInstance()->ARM_SWING_GAIN -= 0.1;
			printf("%.1f    ", Walking::GetInstance()->ARM_SWING_GAIN);
			break;

		case BAL_KNEE_GAIN_ROW:
			if(large == true)
				Walking::GetInstance()->BALANCE_KNEE_GAIN -= 0.1;
			else
				Walking::GetInstance()->BALANCE_KNEE_GAIN -= 0.01;
			printf("%.2f    ", Walking::GetInstance()->BALANCE_KNEE_GAIN);
			break;

		case BAL_ANKLE_PITCH_GAIN_ROW:
			if(large == true)
				Walking::GetInstance()->BALANCE_ANKLE_PITCH_GAIN -= 0.1;
			else
				Walking::GetInstance()->BALANCE_ANKLE_PITCH_GAIN -= 0.01;
			printf("%.2f    ", Walking::GetInstance()->BALANCE_ANKLE_PITCH_GAIN);
			break;

		case BAL_HIP_ROLL_GAIN_ROW:
			if(large == true)
				Walking::GetInstance()->BALANCE_HIP_ROLL_GAIN -= 0.1;
			else
				Walking::GetInstance()->BALANCE_HIP_ROLL_GAIN -= 0.01;
			printf("%.2f    ", Walking::GetInstance()->BALANCE_HIP_ROLL_GAIN);
			break;

		case BAL_ANKLE_ROLL_GAIN_ROW:
			if(large == true)
				Walking::GetInstance()->BALANCE_ANKLE_ROLL_GAIN -= 0.1;
			else
				Walking::GetInstance()->BALANCE_ANKLE_ROLL_GAIN -= 0.01;
			printf("%.2f    ", Walking::GetInstance()->BALANCE_ANKLE_ROLL_GAIN);
			break;

		case P_GAIN_ROW:
			if(large == true)
				Walking::GetInstance()->P_GAIN -= 10;
			else
				Walking::GetInstance()->P_GAIN -= 1;
			printf("%d    ", Walking::GetInstance()->P_GAIN);
			break;

		case I_GAIN_ROW:
			if(large == true)
				Walking::GetInstance()->I_GAIN -= 10;
			else
				Walking::GetInstance()->I_GAIN -= 1;
			printf("%d    ", Walking::GetInstance()->I_GAIN);
			break;

		case D_GAIN_ROW:
			if(large == true)
				Walking::GetInstance()->D_GAIN -= 10;
			else
				Walking::GetInstance()->D_GAIN -= 1;
			printf("%d    ", Walking::GetInstance()->D_GAIN);
			break;
		}
	}
	else
	{
		printf("X");
		if(row == ID_R_HAND_ROW)
		{
			MotionManager::GetInstance()->WriteByte(29, 24, 0, 0);
			MotionManager::GetInstance()->WriteByte(31, 24, 0, 0);
			MotionManager::GetInstance()->WriteByte(33, 24, 0, 0);
		}
		else if(row == ID_L_HAND_ROW)
		{
			MotionManager::GetInstance()->WriteByte(30, 24, 0, 0);
			MotionManager::GetInstance()->WriteByte(32, 24, 0, 0);
			MotionManager::GetInstance()->WriteByte(34, 24, 0, 0);
		}
		else
			MotionManager::GetInstance()->WriteByte(row+1, 562, 0, 0);
	}

	GoToCursor(col, row);
}

void BeginCommandMode()
{
	Old_Col = Col;
	Old_Row = Row;
	ClearCmd();
	GoToCursor(CMD_COL, CMD_ROW);
	bBeginCommandMode = true;
}

void EndCommandMode()
{
	GoToCursor(Old_Col, Old_Row);
	bBeginCommandMode = false;
}

void HelpCmd()
{
	system("clear");
	printf("\n");
	printf(" exit: Exits the program\n");
	printf(" re: Refreshes the screen\n");
	printf(" save: Saves any changes made\n");
	printf(" mon: Monitoring sensor\n");
	printf("\n");
	printf("       Copyright ROBOTIS CO.,LTD.\n");
	printf("\n");
	printf(" Press any key to continue...");
	_getch();

	DrawScreen();
}

void SaveCmd()
{
	bEdited = false;
}

void MonitorCmd()
{
	int col;
	int row;
	int ch;
	int value;
	int GyroFB_min = 1000, GyroFB_max = -1000;
	int GyroRL_min = 1000, GyroRL_max = -1000;
	int AccelFB_min = 1000, AccelFB_max = -1000;
	int AccelRL_min = 1000, AccelRL_max = -1000;

	if(bBeginCommandMode == true)
	{
		col = Old_Col;
		row = Old_Row;
	}
	else
	{
		col = Col;
		row = Row;
	}

	system("clear");
	printf("\n");	
	printf("Gyro F/B                  \n"); // 0
	printf("Gyro R/L                  \n"); // 1
	printf("Accel F/B                 \n"); // 2
	printf("Accel R/L                 \n"); // 3
	printf("ESC (quit), SPACE (reset)   \n");

	set_stdin();
	while(1)
	{
		value = MotionStatus::FB_GYRO; //MotionStatus::FB_GYRO;
		if(GyroFB_min > value)
			GyroFB_min = value;
		if(GyroFB_max < value)
			GyroFB_max = value;
		GoToCursor(PARAM_COL, X_OFFSET_ROW);
		printf("%d (%d~%d)   ", value, GyroFB_min, GyroFB_max);

		value = MotionStatus::RL_GYRO; //MotionStatus::RL_GYRO;
		if(GyroRL_min > value)
			GyroRL_min = value;
		if(GyroRL_max < value)
			GyroRL_max = value;
		GoToCursor(PARAM_COL, Y_OFFSET_ROW);
		printf("%d (%d~%d)   ", value, GyroRL_min, GyroRL_max);

		value = MotionStatus::FB_ACCEL;
		if(AccelFB_min > value)
			AccelFB_min = value;
		if(AccelFB_max < value)
			AccelFB_max = value;
		GoToCursor(PARAM_COL, Z_OFFSET_ROW);
		printf("%d (%d~%d)   ", value, AccelFB_min, AccelFB_max);

		value = MotionStatus::RL_ACCEL;
		if(AccelRL_min > value)
			AccelRL_min = value;
		if(AccelRL_max < value)
			AccelRL_max = value;
		GoToCursor(PARAM_COL, ROLL_OFFSET_ROW);
		printf("%d (%d~%d)   ", value, AccelRL_min, AccelRL_max);

		if(kbhit())
		{
			ch = _getch();
			if(ch == 0x1b) // ESC
				break;
			else if(ch == 0x20) // Space
			{
				GyroFB_min = 1000; GyroFB_max = -1000;
				GyroRL_min = 1000; GyroRL_max = -1000;
				AccelFB_min = 1000; AccelFB_max = -1000;
				AccelRL_min = 1000; AccelRL_max = -1000;
			}
		}

		usleep(50000);
	}
	reset_stdin();
	GoToCursor(col, row);
	DrawScreen();
}
