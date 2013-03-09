#include <math.h>
#include <vector>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <assert.h>
#include <string.h>
#include <ncurses.h>
#include "dcm.h"
#include "config.h"
#include "dynamixel_thread.h"
#include "utils.h"

// comms_manager : communications manager for robotis arm
// author(s) : Mike Hopkins
///////////////////////////////////////////////////////////////////////////

extern Dcm dcm;
extern Config config;
static dynamixel_thread arm_thread;

static void zero_joints();
static void draw_screen();
static void print_joint_array(double *joint_array);

int main(int argc, char **argv)
{
  // initialize communication threads

  // start communication threads
  fprintf(stderr, "starting comms threads...\n");
  arm_thread.start();
  while (!arm_thread.is_running()) {
    usleep(2e5);
  }
  
  arm_thread.set_interface("/dev/ttyUSB0");

  // update display data
  int key = 0;
  initscr();
  cbreak();
  noecho();
  timeout(50);
  while ('q' != (key = getch()))
  {
    clear();
    draw_screen();
    refresh();
    if (!arm_thread.is_running())
       break;
  }
  draw_screen();
  refresh();
  endwin();
  
  // stop communication threads 
  arm_thread.stop();

  return 0;
}

void draw_screen()
{
  // draw display data
  move(0, 0);
  printw("                   Robotis Arm Communications Manager\n");
  printw("///////////////////////////////////////");
  printw("///////////////////////////////////////\n\n");
  
  printw("enable\n");
  print_joint_array(dcm.joint_enable);
  printw("position_p_gain\n");
  print_joint_array(dcm.joint_position_p_gain);
  printw("force\n");
  print_joint_array(dcm.joint_force_sensor);
  printw("position\n");
  print_joint_array(dcm.joint_position_sensor);
  printw("velocity\n");
  print_joint_array(dcm.joint_velocity_sensor);
  printw("thread fps\n");
  printw("            [ %7.0f ]\n", 
    arm_thread.get_fps());
  if (strlen(arm_thread.get_error_message()))
  {
    printw("error\n");
    printw("            [ %s ]\n", arm_thread.get_error_message());
  }
  printw("\npress <q> to quit...");
}

void print_joint_array(double *joint_array)
{
  printw("            [ ");
  for (int i = 0; i < 9; i++)
    printw("%7.3f ", joint_array[i]);
  printw("  ] \n");
}
