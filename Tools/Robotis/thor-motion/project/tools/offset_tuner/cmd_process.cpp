#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <termios.h>
#include <term.h>
#include <fcntl.h>
#include <ncurses.h>
#include "cmd_process.h"

using namespace Thor;


int Col = OFFSET_COL;
int Row = ID_1_ROW;
int Old_Col;
int Old_Row;
bool bBeginCommandMode = false;
bool bEdited = false;
int colnum = 0;

int P_Gain[MotionStatus::MAXIMUM_NUMBER_OF_JOINTS] = { 32, 32, 32, 32, 32, 32,
														32, 32, 32, 32, 32, 32,
														32, 32, 32, 32, 32, 32,
														32, 32, 32, 32, 32, 32,
														32, 32, 32, 32, 32, 32,
														32, 32, 32, 32, 32, 32};

int I_Gain[MotionStatus::MAXIMUM_NUMBER_OF_JOINTS] = { 0, 0, 0, 0, 0, 0,
		                                                0, 0, 0, 0, 0, 0,
		                                                0, 0, 0, 0, 0, 0,
		                                                0, 0, 0, 0, 0, 0,
		                                                0, 0, 0, 0, 0, 0,
		                                                0, 0, 0, 0, 0, 0 };
int D_Gain[MotionStatus::MAXIMUM_NUMBER_OF_JOINTS] = { 0, 0, 0, 0, 0, 0,
														0, 0, 0, 0, 0, 0,
														0, 0, 0, 0, 0, 0,
														0, 0, 0, 0, 0, 0,
														0, 0, 0, 0, 0, 0,
														0, 0, 0, 0, 0, 0 };

//                     1      2       3      4       5        6       7      8      9     10    11    12    13    14    15    16    17    18    19    20
int InitPose[30] = {   0,     0, -70000, 70000, 125500, -125500, -62750, 62750,     0,     0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,
//                    21      22     23     24      25       26      27     28     29     30
		           62750, -62750,     0,     0, -15000,   15000,      0,     0,     0,     0};

unsigned int position[MotionStatus::MAXIMUM_NUMBER_OF_JOINTS];            // Joint position   0~72
unsigned int PresentPos[MotionStatus::MAXIMUM_NUMBER_OF_JOINTS];

const int INVALID_BIT_MASK = 0x40000000;
const int TORQUE_OFF_BIT_MASK = 0x20000000;



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

void ReadStep(MotionManager *manager)
{
	int value;
	int id;
	bool *Enable = new bool[MotionStatus::MAXIMUM_NUMBER_OF_JOINTS];

	for(unsigned int index = 0; index<MotionStatus::MAXIMUM_NUMBER_OF_JOINTS; index++)
	{
		Enable[index] = false;
	}

	for(int jointIndex=0; jointIndex < MotionStatus::MAXIMUM_NUMBER_OF_JOINTS; jointIndex++)
	{
		if(jointIndex < MotionStatus::m_CurrentJoints.size())
		{
			id = MotionStatus::m_CurrentJoints[jointIndex].m_ID;
			if( MotionStatus::m_CurrentJoints[jointIndex].m_DXL_Comm->GetDXLInstance()->ReadByte(id, PRO54::P_TORQUE_ENABLE, &value, 0) == COMM_RXSUCCESS)
			{
				if(value == 1)
				{
					if( MotionStatus::m_CurrentJoints[jointIndex].m_DXL_Comm->GetDXLInstance()->ReadDWord(id, PRO54::P_PRESENT_POSITION_LL, (long*)&value, 0) == COMM_RXSUCCESS)
					{
						position[id-1] = value;
						PresentPos[id-1] = value;
						Enable[id-1] = true;
					}
					else
						position[id-1] = INVALID_BIT_MASK;
				}
				else
				{
					position[id-1] = TORQUE_OFF_BIT_MASK;
					Enable[id-1] = true;
				}
			}
			else
				position[id-1] = INVALID_BIT_MASK;
		}

	}

	for(unsigned index = 0 ; index < MotionStatus::m_CurrentJoints.size(); index++)
	{
		if(Enable[index] == false)
			position[index] = INVALID_BIT_MASK;
	}
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
	if(Col >= GOAL_COL && Col <= D_GAIN_COL)
	{
		if( Row > ID_1_ROW )
			GoToCursor(Col, Row-1);
	}
}

void MoveDownCursor()
{
	if(Col >= GOAL_COL && Col <= D_GAIN_COL)
	{
		if( Row < ID_30_ROW )
			GoToCursor(Col, Row+1);
	}
}

void MoveLeftCursor()
{
	switch(Col)
	{
	case OFFSET_COL:
		GoToCursor(GOAL_COL, Row);
		break;

	case MODVAL_COL:
		GoToCursor(OFFSET_COL, Row);
		break;

	case PRSVAL_COL:
		GoToCursor(MODVAL_COL, Row);
		break;

	case ERRORS_COL:
		GoToCursor(PRSVAL_COL, Row);
		break;

	case P_GAIN_COL:
		GoToCursor(ERRORS_COL, Row);
		break;

	case I_GAIN_COL:
		GoToCursor(P_GAIN_COL, Row);
		break;

	case D_GAIN_COL:
		GoToCursor(I_GAIN_COL, Row);
		break;
	}
}

void MoveRightCursor()
{
	switch(Col)
	{
	case GOAL_COL:
		GoToCursor(OFFSET_COL, Row);
		break;

	case OFFSET_COL:
		GoToCursor(MODVAL_COL, Row);
		break;

	case MODVAL_COL:
		GoToCursor(PRSVAL_COL, Row);
		break;

	case PRSVAL_COL:
		GoToCursor(ERRORS_COL, Row);
		break;

	case ERRORS_COL:
		GoToCursor(P_GAIN_COL, Row);
		break;

	case P_GAIN_COL:
		GoToCursor(I_GAIN_COL, Row);
		break;

	case I_GAIN_COL:
		GoToCursor(D_GAIN_COL, Row);
		break;
	}
}

void DrawIntro(MotionManager *manager)
{
	int id;
	int n = 0;
	int wGoalPosition, wDistance;
	unsigned char *param = new unsigned char[MotionStatus::m_CurrentJoints.size()*9 ];
	for(int jointIndex = 0; jointIndex<MotionStatus::m_CurrentJoints.size(); jointIndex++)
	{
		id = MotionStatus::m_CurrentJoints[jointIndex].m_ID;
		wGoalPosition = InitPose[id-1] + MotionManager::GetInstance()->m_Offset[id-1];

		wDistance = 4000;

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

	manager->m_DXLComm0->GetDXLInstance()->SyncWrite(PRO54::P_GOAL_POSITION_LL, 8, param, MotionStatus::m_CurrentJoints.size()*9 );
	manager->m_DXLComm1->GetDXLInstance()->SyncWrite(PRO54::P_GOAL_POSITION_LL, 8, param, MotionStatus::m_CurrentJoints.size()*9 );
	manager->m_DXLComm2->GetDXLInstance()->SyncWrite(PRO54::P_GOAL_POSITION_LL, 8, param, MotionStatus::m_CurrentJoints.size()*9 );
	manager->m_DXLComm3->GetDXLInstance()->SyncWrite(PRO54::P_GOAL_POSITION_LL, 8, param, MotionStatus::m_CurrentJoints.size()*9 );


	int nrows, ncolumns;
	setupterm(NULL, fileno(stdout), (int *)0);
	nrows = tigetnum("lines");
	ncolumns = tigetnum("cols");

	system("clear");
	printf("\n");
	printf("[Offset Tuner for Thor %s]\n", PROGRAM_VERSION);
	printf("\n");
	printf(" *Terminal screen size must be %d(col)x%d(row).\n", SCREEN_COL, SCREEN_ROW);
	printf(" *Current terminal has %d columns and %d rows.\n", ncolumns, nrows);
	printf("\n");
	printf("\n");
	printf("Please wait 5 seconds\n");


	ReadStep(manager);

	sleep(5);
	n= 0;
	for(int jointIndex = 0; jointIndex<MotionStatus::m_CurrentJoints.size(); jointIndex++)
	{
		id = MotionStatus::m_CurrentJoints[jointIndex].m_ID;
		wGoalPosition = InitPose[id-1] + MotionManager::GetInstance()->m_Offset[id-1];

		wDistance = 17000;

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
	manager->m_DXLComm0->GetDXLInstance()->SyncWrite(PRO54::P_GOAL_POSITION_LL, 8, param, MotionStatus::m_CurrentJoints.size()*9 );
	manager->m_DXLComm1->GetDXLInstance()->SyncWrite(PRO54::P_GOAL_POSITION_LL, 8, param, MotionStatus::m_CurrentJoints.size()*9 );
	manager->m_DXLComm2->GetDXLInstance()->SyncWrite(PRO54::P_GOAL_POSITION_LL, 8, param, MotionStatus::m_CurrentJoints.size()*9 );
	manager->m_DXLComm3->GetDXLInstance()->SyncWrite(PRO54::P_GOAL_POSITION_LL, 8, param, MotionStatus::m_CurrentJoints.size()*9 );

	delete[] param;

	printf("Press any key to start program...\n");
	_getch();

	DrawPage();
}

void DrawEnding()
{
	system("clear");
	printf("\n");
	printf("Terminate Offset tuner");
	printf("\n");
}

void DrawPage()
{
	int old_col = Col;
	int old_row = Row;

	system("clear");
	//       0         1         2         3         4         5         6         7         8         9
	// 80    01234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567     //24
	printf( "ID: 1(R_SHO_PITCH)  [       ]          [       ]|                                        \n" );//0
	printf( "ID: 2(L_SHO_PITCH)  [       ]          [       ]|                                        \n" );//1
	printf( "ID: 3(R_SHO_ROLL)   [       ]          [       ]|                                        \n" );//2
	printf( "ID: 4(L_SHO_ROLL)   [       ]          [       ]|                                        \n" );//3
	printf( "ID: 5(R_SHO_YAW)    [       ]          [       ]|                                        \n" );//4
	printf( "ID: 6(L_SHO_YAW)    [       ]          [       ]|                                        \n" );//5
	printf( "ID: 7(R_ELBOW)      [       ]          [       ]|                                        \n" );//6
	printf( "ID: 8(L_ELBOW)      [       ]          [       ]|                                        \n" );//7
	printf( "ID: 9(R_WRI_YAW1)   [       ]          [       ]|                                        \n" );//8
	printf( "ID:10(L_WRI_YAW1)   [       ]          [       ]|                                        \n" );//9
	printf( "ID:11(R_WRI_ROLL)   [       ]          [       ]|                                        \n" );//0
	printf( "ID:12(L_WRI_ROLL)   [       ]          [       ]|                                        \n" );//1
	printf( "ID:13(R_WRI_YAW2)   [       ]          [       ]|                                        \n" );//2
	printf( "ID:14(L_WRI_YAW2)   [       ]          [       ]|                                        \n" );//3
	printf( "ID:15(R_HIP_YAW)    [       ]          [       ]|                                        \n" );//4
	printf( "ID:16(L_HIP_YAW)    [       ]          [       ]|                                        \n" );//5
	printf( "ID:17(R_HIP_ROLL)   [       ]          [       ]|                                        \n" );//6
	printf( "ID:18(L_HIP_ROLL)   [       ]          [       ]|                                        \n" );//7
	printf( "ID:19(R_HIP_PITCH)  [       ]          [       ]|                                        \n" );//8
	printf( "ID:20(L_HIP_PITCH)  [       ]          [       ]|                                        \n" );//9
	printf( "ID:21(R_KNEE)       [       ]          [       ]|                                        \n" );//0
	printf( "ID:22(L_KNEE)       [       ]          [       ]|                                        \n" );//1
	printf( "ID:23(R_ANK_PITCH)  [       ]          [       ]|                                        \n" );//2
	printf( "ID:24(L_ANK_PITCH)  [       ]          [       ]|                                        \n" );//3
	printf( "ID:25(R_ANK_ROLL)   [       ]          [       ]|                                        \n" );//4
	printf( "ID:26(L_ANK_ROLL)   [       ]          [       ]|                                        \n" );//5
	printf( "ID:27(WAIST_PAN)    [       ]          [       ]|                                        \n" );//6
	printf( "ID:28(WAIST_TILT)   [       ]          [       ]|                                        \n" );//7
	printf( "ID:29(HEAD_PAN)     [       ]          [       ]|                                        \n" );//8
	printf( "ID:30(HEAD_TILT)    [       ]          [       ]|                                        \n" );//9
	printf( "                    GOAL      OFFSET   MODVAL    PRSPOS  ERRORS   P_GAIN  I_GAIN   D_GAIN\n" );//0
	printf( "]                                                                                        " );  //1


	for(int i=0; i<=7; i++ )
		DrawStep(i);

	GoToCursor(old_col, old_row);
}

void DrawStep(int index)
{
	int old_col = Col;
	int old_row = Row;
	int col;

	switch(index)
	{
	case 0:
		col = OFFSET_COL;
		for( unsigned int jointIndex = 0; jointIndex<MotionStatus::m_CurrentJoints.size(); jointIndex++ )
		{
			int id = -1;
			id = MotionStatus::m_CurrentJoints[jointIndex].m_ID;
			GoToCursor(col, id -1);
			printf("%-7d ", MotionManager::GetInstance()->m_Offset[id-1]);
		}
		break;

	case 1:
		col = MODVAL_COL;
		for( unsigned int jointIndex = 0; jointIndex<MotionStatus::m_CurrentJoints.size(); jointIndex++ )
		{
			int id = -1;
			id = MotionStatus::m_CurrentJoints[jointIndex].m_ID;
			GoToCursor(col, id -1);
			printf("%-7d", PresentPos[id-1]);
		}
		break;

	case 2:
		col = PRSVAL_COL;
		for( unsigned int jointIndex = 0; jointIndex<MotionStatus::m_CurrentJoints.size(); jointIndex++  )
		{
			int id = -1;
			id = MotionStatus::m_CurrentJoints[jointIndex].m_ID;
			GoToCursor(col, id-1);
			int status = 0;
			CheckServoStatus(id, &status);
			if(status == INVALID_BIT_MASK)
				printf("-------");
			else if(status == TORQUE_OFF_BIT_MASK)
				printf("???????");
			else
				printf("%-7d", PresentPos[id-1]);
		}
		break;

	case 3:
		col = ERRORS_COL;
		for( unsigned int jointIndex = 0; jointIndex<MotionStatus::m_CurrentJoints.size(); jointIndex++ )
		{
			int id = -1;
			id = MotionStatus::m_CurrentJoints[jointIndex].m_ID;
			GoToCursor(col, id -1);
			int status = 0;
			CheckServoStatus(id, &status);
			if(status == INVALID_BIT_MASK)
				printf("-------");
			else if(status == TORQUE_OFF_BIT_MASK)
				printf("???????");
			else
				printf("%-7d ", PresentPos[id-1] - position[id-1]);
		}
		break;

	case 4:
		col = GOAL_COL;
		for(unsigned int jointIndex = 0; jointIndex<MotionStatus::m_CurrentJoints.size(); jointIndex++ )
		{
			int id = -1;
			id = MotionStatus::m_CurrentJoints[jointIndex].m_ID;
			GoToCursor(col, id -1);
			printf("%-7d", InitPose[id-1]);
		}
		break;

	case 5:
		col = P_GAIN_COL;
		for(unsigned int jointIndex = 0; jointIndex<MotionStatus::m_CurrentJoints.size(); jointIndex++ )
		{
			int id = -1;
			id = MotionStatus::m_CurrentJoints[jointIndex].m_ID;
			GoToCursor(col, id -1);
			printf("%-6d", P_Gain[id-1]);
		}
		break;

	case 6:
		col = I_GAIN_COL;
		for( unsigned int jointIndex = 0; jointIndex<MotionStatus::m_CurrentJoints.size(); jointIndex++ )
		{
			int id = -1;
			id = MotionStatus::m_CurrentJoints[jointIndex].m_ID;
			GoToCursor(col, id -1);
			printf("%-6d", I_Gain[id-1]);
		}
		break;

	case 7:
		col = D_GAIN_COL;
		for( unsigned int jointIndex = 0; jointIndex<MotionStatus::m_CurrentJoints.size(); jointIndex++ )
		{
			int id = -1;
			id = MotionStatus::m_CurrentJoints[jointIndex].m_ID;
			GoToCursor(col, id -1);
			printf("%-6d", D_Gain[id-1]);
		}
		break;

	default:
		return;
	}

	GoToCursor( old_col, old_row );
}

void DrawStepLine(bool erase)
{
	int old_col = Col;
	int old_row = Row;
	int col;

	switch(colnum)
	{
	case 0:
		col = OFFSET_COL;
		break;

	case 1:
		col = MODVAL_COL;
		break;

	case 2:
		col = PRSVAL_COL;
		break;

	case 3:
		col = ERRORS_COL;
		break;

	default:
		return;
	}
	col--;

	for( unsigned int jointIndex = 0; jointIndex<MotionStatus::m_CurrentJoints.size(); jointIndex++ )
	{
		int id = -1;
		id = MotionStatus::m_CurrentJoints[jointIndex].m_ID;
		GoToCursor(col, id - 1);
		if(erase == true)
			printf( " " );
		else
			printf( "|" );
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

void UpDownValue(MotionManager *manager, int offset)
{
	if(Col == OFFSET_COL)
	{
		int id = -1;
		unsigned int jointIndex = 0;
		for(jointIndex = 0 ; jointIndex<MotionStatus::m_CurrentJoints.size(); jointIndex++)
		{
			id = MotionStatus::m_CurrentJoints[jointIndex].m_ID;
			if(id == Row+1)
				break;
		}

		if((InitPose[Row] + GetValue() + offset) > MotionStatus::m_CurrentJoints[jointIndex].m_DXLInfo->MAX_VALUE)
			SetValue(manager, MotionStatus::m_CurrentJoints[jointIndex].m_DXLInfo->MAX_VALUE - InitPose[Row]);
		else if((InitPose[Row] + GetValue() + offset) < MotionStatus::m_CurrentJoints[jointIndex].m_DXLInfo->MIN_VALUE)
			SetValue(manager, MotionStatus::m_CurrentJoints[jointIndex].m_DXLInfo->MIN_VALUE - InitPose[Row]);
		else
			SetValue(manager, GetValue() + offset);
		bEdited = true;
	}
	else
		SetValue(manager, GetValue() + offset);
}

int GetValue()
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

	if( col == GOAL_COL )
		return InitPose[row];
	else if( col == OFFSET_COL )
		return MotionManager::GetInstance()->m_Offset[row];
	else if( col == MODVAL_COL )
		return position[row];
	else if( col == PRSVAL_COL )
		return PresentPos[row];
	else if( col == ERRORS_COL )
		return PresentPos[row] - position[row];
	else if( col == P_GAIN_COL )
		return P_Gain[row];
	else if( col == I_GAIN_COL )
		return I_Gain[row];
	else if( col == D_GAIN_COL )
		return D_Gain[row];

	return -1;
}

void SetValue(MotionManager *manager, int value)
{
	int col;
	int row;
	int status;
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

	GoToCursor(col, row);

	int id = -1;
	unsigned int jointIndex = 0;
	for(jointIndex = 0 ; jointIndex<MotionStatus::m_CurrentJoints.size(); jointIndex++)
	{
		id = MotionStatus::m_CurrentJoints[jointIndex].m_ID;
		if(id == row+1)
			break;
	}

	if( col == GOAL_COL )
	{
		if(value+MotionManager::GetInstance()->m_Offset[row] >= MotionStatus::m_CurrentJoints[jointIndex].m_DXLInfo->MIN_VALUE
				&& value+MotionManager::GetInstance()->m_Offset[row] <= MotionStatus::m_CurrentJoints[jointIndex].m_DXLInfo->MAX_VALUE)
		{
			if( CheckServoStatus(row+1, &status) )
			{
				int error;
				if(manager->WriteDWord(row+1, PRO54::P_GOAL_POSITION_LL, value + MotionManager::GetInstance()->m_Offset[row], &error) == COMM_RXSUCCESS)
				{
					if(!(error & ERRBIT_ANGLE))
					{
						InitPose[row] = value;
						printf( "%-7d", value );
						position[row] = value+MotionManager::GetInstance()->m_Offset[row];
						GoToCursor(MODVAL_COL, row);
						printf( "%-7d", position[row] );
					}
				}
			}
		}
	}
	else if( col == OFFSET_COL )
	{
		MotionManager::GetInstance()->m_Offset[row] = value;
		printf( "%-7d ", GetValue() );

		if(InitPose[row] + value >= MotionStatus::m_CurrentJoints[jointIndex].m_DXLInfo->MIN_VALUE
				&& InitPose[row] + value <= MotionStatus::m_CurrentJoints[jointIndex].m_DXLInfo->MAX_VALUE)
		{
			if(CheckServoStatus(row+1, &status))
			{
				int error;
				if(manager->WriteDWord(row+1, PRO54::P_GOAL_POSITION_LL, InitPose[row] + value, &error) == COMM_RXSUCCESS)
				{
					if(!(error & ERRBIT_ANGLE))
					{
						position[row] = InitPose[row] + value;
						GoToCursor(MODVAL_COL, row);
						printf( "%-7d", position[row] );
					}
				}
			}
		}
		bEdited = true;
	}
	else if( col == MODVAL_COL )
	{
		if(value >= MotionStatus::m_CurrentJoints[jointIndex].m_DXLInfo->MIN_VALUE
				&& value <= MotionStatus::m_CurrentJoints[jointIndex].m_DXLInfo->MAX_VALUE)
		{
			if(CheckServoStatus(row+1, &status))
			{
				int error;
				if(manager->WriteDWord(row + 1, PRO54::P_GOAL_POSITION_LL, value, &error) == COMM_RXSUCCESS)
				{
					if(!(error & ERRBIT_ANGLE))
					{
						position[row] = value;
						printf( "%-7d", position[row] );
						MotionManager::GetInstance()->m_Offset[row] = value - InitPose[row];
						GoToCursor(OFFSET_COL, row);
						printf( "%-7d ", MotionManager::GetInstance()->m_Offset[row] );
					}
				}
			}
		}
	}
	else if( col == PRSVAL_COL )
	{
		printf( "%-7d", GetValue());
	}
	else if( col == ERRORS_COL )
	{
		printf( "%-7d ", GetValue());
	}
	else if( col == P_GAIN_COL )
	{
		if(value >= 0 && value <= 65535)
		{
			if(CheckServoStatus(row+1, &status) )
			{
				int error;
				if(manager->WriteWord(row + 1, PRO54::P_POSITION_P_GAIN_L, value, &error) == COMM_RXSUCCESS)
				{
					P_Gain[row] = value;
					printf( "%-6d", P_Gain[row] );
				}
			}
		}
	}
	else if( col == I_GAIN_COL )
	{
		if(value >= 0 && value <= 65535)
		{
			if(CheckServoStatus(row+1, &status) )
			{
				int error;
				if(manager->WriteWord(row + 1, PRO54::P_POSITION_I_GAIN_L, value, &error) == COMM_RXSUCCESS)
				{
					I_Gain[row] = value;
					printf( "%-6d", I_Gain[row] );
				}
			}
		}
	}
	else if( col == D_GAIN_COL )
	{
		if(value >= 0 && value <= 65535)
		{
			if(CheckServoStatus(row+1, &status) )
			{
				int error;
				if(manager->WriteWord(row + 1, PRO54::P_POSITION_D_GAIN_L, value, &error) == COMM_RXSUCCESS)
				{
					D_Gain[row] = value;
					printf( "%-6d", D_Gain[row] );
				}
			}
		}
	}

	GoToCursor(col, row);	
}

void ToggleTorque(MotionManager *manager)
{
	if((Col != GOAL_COL && Col != MODVAL_COL) || Row > ID_20_ROW)
		return;

	int id = Row + 1;
	int status;
	CheckServoStatus(id, &status);

	if(status == TORQUE_OFF_BIT_MASK)
	{
		if(manager->WriteByte(id, PRO54::P_TORQUE_ENABLE, 1, 0) != COMM_RXSUCCESS)
			return;

		int value;
		if(Col == MODVAL_COL)
		{
			if(manager->ReadDWord(id, PRO54::P_GOAL_POSITION_LL, (long*)&value, 0) != COMM_RXSUCCESS)
				return;
			position[id-1] = value;
		}
		else if(Col == PRSVAL_COL)
		{
			if(manager->ReadDWord(id, PRO54::P_GOAL_POSITION_LL, (long*)&value, 0) != COMM_RXSUCCESS)
				return;
			position[id-1] = value;

			if(manager->ReadDWord(id, PRO54::P_PRESENT_POSITION_LL, (long*)&value, 0) != COMM_RXSUCCESS)
				return;
			PresentPos[id-1] = value;
		}
		else if(Col == GOAL_COL)
		{
			if(manager->ReadDWord(id, PRO54::P_GOAL_POSITION_LL, (long*)&value, 0) != COMM_RXSUCCESS)
				return;
			position[id-1] = value;
			InitPose[id-1] = value = position[id-1] - MotionManager::GetInstance()->m_Offset[id-1];
		}

		//printf("%.4d", value);
		SetValue(manager, value);
	}
	else
	{
		if(manager->WriteByte(id, PRO54::P_GOAL_POSITION_LL, 0, 0) != COMM_RXSUCCESS)
			return;

		position[id-1] = TORQUE_OFF_BIT_MASK;
		printf("???????");
	}

	GoToCursor(Col, Row);
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
	printf(" exit               Exits the program.\n");
	printf(" re                 Refreshes the screen.\n");
	printf(" set [value]        Sets value on cursor [value].\n");
	printf(" pgain [value]      Sets ALL actuators' P gain to [value].\n");
	printf(" igain [value]      Sets ALL actuators' I gain to [value].\n");
	printf(" dgain [value]      Sets ALL actuators' D gain to [value].\n");
	printf(" save               Saves offset changes.\n");
	printf(" on/off             Turn On/Off torque from ALL actuators.\n");
	printf(" on/off [index1] [index2] ...  \n"
			"                    turns On/Off torque from ID[index1] ID[index2]...\n");
	printf("\n");
	printf("       Copyright ROBOTIS CO.,LTD.\n");
	printf("\n");
	printf(" Press any key to continue...");
	_getch();

	DrawPage();
}

void OnOffCmd(MotionManager *manager, bool on, int num_param, int *list)
{
	if(num_param == 0)
	{
		int id = -1;
		for(unsigned int jointIndex = 0; jointIndex < MotionStatus::m_CurrentJoints.size(); jointIndex++ )
		{
			id = MotionStatus::m_CurrentJoints[jointIndex].m_ID;
			manager->WriteByte(id, PRO54::P_TORQUE_ENABLE, (int)on, 0);
		}

	}
	else
	{
		int id = -1;
		for(int i=0; i<num_param; i++)
		{
			id = list[i];
			manager->WriteByte(list[i], PRO54::P_TORQUE_ENABLE, (int)on, 0);
		}
	}

	ReadStep(manager);
	//DrawStep(7);
	DrawPage();
}


void GainCmd(MotionManager *manager, int value, int pid_col)
{
	if(value < 0 || value > 65535)
	{
		PrintCmd("Invalid gain value");
		return;
	}

	int id = -1;
	int n = 0;
	unsigned char param[3 * MotionStatus::m_CurrentJoints.size()];

	for(unsigned int jointIndex = 0; jointIndex < MotionStatus::m_CurrentJoints.size(); jointIndex++)
	{
		id = MotionStatus::m_CurrentJoints[jointIndex].m_ID;
		param[n++] = (unsigned char)id;
		param[n++] = DXL_LOBYTE(value);
		param[n++] = DXL_HIBYTE(value);

		if(pid_col == P_GAIN_COL)       P_Gain[id-1] = value;
		else if(pid_col == I_GAIN_COL)  I_Gain[id-1] = value;
		else if(pid_col == D_GAIN_COL)  D_Gain[id-1] = value;
	}

	if(pid_col == P_GAIN_COL)
	{
		manager->m_DXLComm0->GetDXLInstance()->SyncWrite(PRO54::P_POSITION_P_GAIN_L, 2, param, 3* MotionStatus::m_CurrentJoints.size());
		manager->m_DXLComm1->GetDXLInstance()->SyncWrite(PRO54::P_POSITION_P_GAIN_L, 2, param, 3* MotionStatus::m_CurrentJoints.size());
		manager->m_DXLComm2->GetDXLInstance()->SyncWrite(PRO54::P_POSITION_P_GAIN_L, 2, param, 3* MotionStatus::m_CurrentJoints.size());
		manager->m_DXLComm3->GetDXLInstance()->SyncWrite(PRO54::P_POSITION_P_GAIN_L, 2, param, 3* MotionStatus::m_CurrentJoints.size());
		DrawStep(5);
	}
	else if(pid_col == I_GAIN_COL)
	{
		manager->m_DXLComm0->GetDXLInstance()->SyncWrite(PRO54::P_POSITION_I_GAIN_L, 2, param, 3* MotionStatus::m_CurrentJoints.size());
		manager->m_DXLComm1->GetDXLInstance()->SyncWrite(PRO54::P_POSITION_I_GAIN_L, 2, param, 3* MotionStatus::m_CurrentJoints.size());
		manager->m_DXLComm2->GetDXLInstance()->SyncWrite(PRO54::P_POSITION_I_GAIN_L, 2, param, 3* MotionStatus::m_CurrentJoints.size());
		manager->m_DXLComm3->GetDXLInstance()->SyncWrite(PRO54::P_POSITION_I_GAIN_L, 2, param, 3* MotionStatus::m_CurrentJoints.size());
		DrawStep(6);
	}
	else if(pid_col == D_GAIN_COL)
	{
		manager->m_DXLComm0->GetDXLInstance()->SyncWrite(PRO54::P_POSITION_D_GAIN_L, 2, param, 3* MotionStatus::m_CurrentJoints.size());
		manager->m_DXLComm1->GetDXLInstance()->SyncWrite(PRO54::P_POSITION_D_GAIN_L, 2, param, 3* MotionStatus::m_CurrentJoints.size());
		manager->m_DXLComm2->GetDXLInstance()->SyncWrite(PRO54::P_POSITION_D_GAIN_L, 2, param, 3* MotionStatus::m_CurrentJoints.size());
		manager->m_DXLComm3->GetDXLInstance()->SyncWrite(PRO54::P_POSITION_D_GAIN_L, 2, param, 3* MotionStatus::m_CurrentJoints.size());
		DrawStep(7);
	}
}

void SaveCmd(minIni *ini)
{
	if(bEdited == false)
		return;

	MotionManager::GetInstance()->SaveINISettings(ini);
	bEdited = false;
}

bool CheckServoStatus(int id, int *status)
{
	*status = 0;
	if(  ( (position[id-1]>>28) == 15 ) || ( (position[id-1] >> 28) == 0 ) )
		return true;
	else if((position[id-1]>>28) == 2)
	{
		*status = TORQUE_OFF_BIT_MASK;
		return false;
	}
	else if((position[id-1]>>28) == 4)
	{
		*status = INVALID_BIT_MASK;
		return false;
	}
	else
		return false;
}

bool CheckPresentStatus(int id, int *status)
{
	*status = 0;
	if(  ( (PresentPos[id-1]>>28) == 15 ) || ( (PresentPos[id-1] >> 28) == 0 ) )
		return true;
	else if((position[id-1]>>28) == 2)
	{
		*status = TORQUE_OFF_BIT_MASK;
		return false;
	}
	else if((position[id-1]>>28) == 4)
	{
		*status = INVALID_BIT_MASK;
		return false;
	}
	else
		return false;
}
