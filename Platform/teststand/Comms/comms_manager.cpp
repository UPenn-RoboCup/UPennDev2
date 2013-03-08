#include <math.h>
#include <vector>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <assert.h>
#include <string.h>
#include <ncurses.h>
#include "epos_thread.h"
#include "config.h"
#include "utils.h"
#include "dcm.h"

// comms_manager : communications manager for actuator teststand
// author: Mike Hopkins
///////////////////////////////////////////////////////////////////////////

extern Dcm dcm;
extern Config config;
static epos_thread actuator_thread;

static void zero_joints();
static void draw_screen();

int main(int argc, char **argv)
{
  // initialize communication threads
  std::vector<int> actuator_joints(1, 0);
  actuator_thread.set_can_interface("can0");
  actuator_thread.set_joints(actuator_joints);

  // start communication threads
  fprintf(stderr, "starting comms threads...\n");
  actuator_thread.start();
  while (!actuator_thread.is_running())
  {
    usleep(2e5);
  }

  // move joints to zero position
  fprintf(stderr, "zeroing joints...\n");
  zero_joints();

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

void zero_joints()
{
  // move all joints to zero position
  double q0[N_JOINT];
  double t0 = get_time();
  double t = t0;
  for (int i = 0; i < N_JOINT; i++)
  {
    q0[i] = dcm.joint_position_sensor[i];
  }
  while (t - t0 < 1)
  {
    t = get_time();
    for (int i = 0; i < N_JOINT; i++)
    {
      dcm.joint_position[i] = (1 - min(t - t0, 1))*q0[i];
    }
  }
}

void draw_screen()
{
  // draw display data
  move(0, 0);
  printw("                       Teststand Communications Manager\n");
  printw("///////////////////////////////////////");
  printw("///////////////////////////////////////\n\n");
  
  printw("joint enable             [ %10.3f ]\n", dcm.joint_enable[0]);
  printw("joint force              [ %10.3f ]\n", dcm.joint_force[0]);
  printw("joint position           [ %10.3f ]\n", dcm.joint_position[0]);
  printw("joint velocity           [ %10.3f ]\n", dcm.joint_velocity[0]);
  printw("joint position_p_gain    [ %10.3f ]\n", dcm.joint_position_p_gain[0]);

  printw("joint force sensor       [ %10.3f ]\n", dcm.joint_force_sensor[0]);
  printw("joint position sensor    [ %10.3f ]\n", dcm.joint_position_sensor[0]);
  printw("joint velocity sensor    [ %10.3f ]\n", dcm.joint_velocity_sensor[0]);
  printw("motor force sensor       [ %10.3f ]\n", dcm.motor_force_sensor[0]);
  printw("motor position sensor    [ %10.3f ]\n", dcm.motor_position_sensor[0]);
  printw("motor velocity sensor    [ %10.3f ]\n", dcm.motor_velocity_sensor[0]);
  printw("motor current sensor     [ %10.3f ]\n", dcm.motor_current_sensor[0]);
  printw("motor temperature sensor [ %10.3f ]\n\n", dcm.motor_temperature_sensor[0]);

  printw("actuator thread fps      [ %10.3f ]\n", actuator_thread.get_fps());
  if (strlen(actuator_thread.get_error_message()))
  {
    printw("actuator thread error    [ %s ]\n", actuator_thread.get_error_message());
  }
  printw("\npress <q> to quit...");

}
