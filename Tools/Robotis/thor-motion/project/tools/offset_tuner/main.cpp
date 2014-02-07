#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <termios.h>
#include <term.h>
#include <ncurses.h>
#include <libgen.h>
#include <signal.h>
#include "cmd_process.h"


#define INI_FILE_PATH       "config.ini"

using namespace Thor;

void change_current_dir()
{
    char exepath[1024] = {0};
    if(readlink("/proc/self/exe", exepath, sizeof(exepath)) != -1)
        chdir(dirname(exepath));
}

void sighandler(int sig)
{
    struct termios term;
    tcgetattr( STDIN_FILENO, &term );
    term.c_lflag |= ICANON | ECHO;
    tcsetattr( STDIN_FILENO, TCSANOW, &term );

    exit(0);
}

int main(int argc, char *argv[])
{
    signal(SIGABRT, &sighandler);
    signal(SIGTERM, &sighandler);
    signal(SIGQUIT, &sighandler);
    signal(SIGINT, &sighandler);

    int ch;
    char filename[128];

    minIni* ini = new minIni(INI_FILE_PATH);

    MotionManager *manager = MotionManager::GetInstance();
    //////////////////// Framework Initialize ////////////////////////////
    if(manager->Initialize() == false)
    {
        printf("Initializing Motion Manager failed!\n");
        return 0;
    }

//    if(manager->InitforTest() == false)
//    {
//        printf("Initializing Motion Manager for Test failed!\n");
//        return 0;
//    }

    for(unsigned int i = 0; i < MotionStatus::m_CurrentJoints.size(); i++)
    {
    	int id = -1;
    	if(id >= 15 && id <= 26)
    		MotionStatus::m_CurrentJoints[i].m_DXL_Comm->GetDXLInstance()->WriteWord(id, PRO54::P_VELOCITY_I_GAIN_L, 16,0);
    }
    /////////////////////////////////////////////////////////////////////

    MotionManager::GetInstance()->LoadINISettings(ini);

    DrawIntro(manager);

    while(1)
    {
        ch = _getch();

        if(ch == 0x1b)
        {
            ch = _getch();
            if(ch == 0x5b)
            {
                ch = _getch();
                if(ch == 0x41)      // Up arrow key
                    MoveUpCursor();
                else if(ch == 0x42) // Down arrow key
                    MoveDownCursor();
                else if(ch == 0x44) // Left arrow key
                    MoveLeftCursor();
                else if(ch == 0x43) // Right arrow key
                    MoveRightCursor();
            }
        }
        else if( ch == '[' )
            UpDownValue(manager, -10);
        else if( ch == ']' )
            UpDownValue(manager, 10);
        else if( ch == '{' )
            UpDownValue(manager, -100);
        else if( ch == '}' )
            UpDownValue(manager, 100);
        else if( ch == ' ' )
            ToggleTorque(manager);
        else if( ch >= 'A' && ch <= 'z' )
        {
            char input[128] = {0,};
            char *token;
            int input_len;
            char cmd[80];
            int num_param;
            int iparam[30];

            int idx = 0;

            BeginCommandMode();

            printf("%c", ch);
            input[idx++] = (char)ch;

            while(1)
            {
                ch = _getch();
                if( ch == 0x0A )
                    break;
                else if( ch == 0x7F )
                {
                    if(idx > 0)
                    {
                        ch = 0x08;
                        printf("%c", ch);
                        ch = ' ';
                        printf("%c", ch);
                        ch = 0x08;
                        printf("%c", ch);
                        input[--idx] = 0;
                    }
                }
                else if( ( ch >= 'A' && ch <= 'z' ) || ch == ' ' || ch == '-' || ( ch >= '0' && ch <= '9'))
                {
                    if(idx < 127)
                    {
                        printf("%c", ch);
                        input[idx++] = (char)ch;
                    }
                }
            }

            fflush(stdin);
            input_len = strlen(input);
            if(input_len > 0)
            {
                token = strtok( input, " " );
                if(token != 0)
                {
                    strcpy( cmd, token );
                    token = strtok( 0, " " );
                    num_param = 0;
                    while(token != 0)
                    {
                        iparam[num_param++] = atoi(token);
                        token = strtok( 0, " " );
                    }

                    if(strcmp(cmd, "exit") == 0)
                    {
                        if(AskSave() == false)
                            break;
                    }
                    else if(strcmp(cmd, "re") == 0)
                    {
                        ReadStep(manager);
                        DrawPage();
                    }
                    else if(strcmp(cmd, "help") == 0)
                        HelpCmd();
                    else if(strcmp(cmd, "set") == 0)
                    {
                        if(num_param > 0)
                            SetValue(manager, iparam[0]);
                        else
                            PrintCmd("Need parameter");
                    }
                    else if(strcmp(cmd, "on") == 0)
                        OnOffCmd(manager, true, num_param, iparam);
                    else if(strcmp(cmd, "off") == 0)
                        OnOffCmd(manager, false, num_param, iparam);
                    else if(strcmp(cmd, "save") == 0)
                        SaveCmd(ini);
                    else if(strcmp(cmd, "pgain") == 0)
                    {
                        if(num_param > 0)
                            GainCmd(manager, iparam[0], P_GAIN_COL);
                        else
                            PrintCmd("Need parameter");
                    }
                    else if(strcmp(cmd, "igain") == 0)
                    {
                        if(num_param > 0)
                            GainCmd(manager, iparam[0], I_GAIN_COL);
                        else
                            PrintCmd("Need parameter");
                    }
                    else if(strcmp(cmd, "dgain") == 0)
                    {
                        if(num_param > 0)
                            GainCmd(manager, iparam[0], D_GAIN_COL);
                        else
                            PrintCmd("Need parameter");
                    }
                    else
                        PrintCmd("Bad command! please input 'help'");
                }
            }

            EndCommandMode();
        }
    }

    DrawEnding();

//    for(unsigned int i = 0; i < MotionStatus::m_CurrentJoints.size(); i++)
//    {
//    	int id = -1;
//    	if(id > 14 && id < 27)
//    		MotionStatus::m_CurrentJoints[i].m_DXL_Comm->GetDXLInstance()->WriteWord(id, PRO54::P_VELOCITY_I_GAIN_L, 0, 0);
//    }

    return 0;
}
