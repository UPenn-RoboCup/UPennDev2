#include <math.h>
#include <vector>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <assert.h>
#include <string.h>
#include <ncurses.h>
#include "slug_thread.h"
#include "config.h"
#include "utils.h"
#include "dcm.h"

// comms_manager : communications manager for actuator teststand
// author: Mike Hopkins
///////////////////////////////////////////////////////////////////////////

Dcm dcm;
Config config;
static slug_thread actuator_thread;
static void draw_screen();

int main(int argc, char **argv)
{
  // initialize communication threads
  actuator_thread.set_can_interface("can0");

  // start communication threads
  fprintf(stderr, "starting comms threads...\n");
  actuator_thread.start();
  while (!actuator_thread.is_running())
  {
    usleep(2e5);
  }

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
  }
  endwin();
  
  // stop comm threads 
  actuator_thread.stop();
  return 0;
}

void draw_screen()
{
  // draw display data
  move(0, 0);
  printw("                       Teststand Communications Manager\n");
  printw("///////////////////////////////////////");
  printw("///////////////////////////////////////\n");
  printw("joint enable             [ %10.3f ]\n", dcm.joint_enable[0]);
  printw("joint force              [ %10.3f ]\n", dcm.joint_force[0]);
  printw("joint position           [ %10.3f ]\n", dcm.joint_position[0]);
  printw("joint velocity           [ %10.3f ]\n", dcm.joint_velocity[0]);
  printw("joint p_gain             [ %10.3f ]\n", dcm.joint_p_gain[0]);
  printw("joint i_gain             [ %10.3f ]\n", dcm.joint_i_gain[0]);
  printw("joint d_gain             [ %10.3f ]\n", dcm.joint_d_gain[0]);
  printw("joint force sensor       [ %10.3f ]\n", dcm.joint_force_sensor[0]);
  printw("joint position sensor    [ %10.3f ]\n", dcm.joint_position_sensor[0]);
  printw("joint velocity sensor    [ %10.3f ]\n", dcm.joint_velocity_sensor[0]);
  printw("actuator thread fps      [ %10.3f ]\n", actuator_thread.get_fps());
  if (strlen(actuator_thread.get_error_message()))
  {
    printw("actuator thread error    [ %s ]\n", actuator_thread.get_error_message());
  }
  printw("\npress <q> to quit...");

}
