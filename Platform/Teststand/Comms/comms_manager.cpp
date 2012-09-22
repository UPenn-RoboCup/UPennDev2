#include <math.h>
#include <vector>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <assert.h>
#include <string.h>
#include <ncurses.h>
#include "shared_data.h"
#include "epos_thread.h"
#include "utils.h"

// comms_manager : communications manager for actuator teststand
// author: Mike Hopkins
///////////////////////////////////////////////////////////////////////////

static void zero_joints();
static void draw_screen();

using namespace shared_data;

namespace shared_data {
  extern struct actuator_data actuator;
  extern struct sensor_data sensor;
  extern struct bias_data bias;
};

namespace {
  epos_thread actuator_thread;
};

int main(int argc, char **argv)
{
  // initialize shared data
  shared_data::entry();

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

  // destroy shared memory
  shared_data::exit();

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
    q0[i] = sensor.joint_position[i];
  }
  while (t - t0 < 1)
  {
    t = get_time();
    for (int i = 0; i < N_JOINT; i++)
    {
      actuator.joint_position[i] = (1 - min(t - t0, 1))*q0[i];
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
  
/*
  printw("joint position bias      [ %10.3f ]\n", bias.joint_position[0]);
  printw("joint force bias         [ %10.3f ]\n", bias.joint_force[0]);
  printw("motor position bias      [ %10.3f ]\n", bias.motor_position[0]);
  printw("motor force bias         [ %10.3f ]\n", bias.motor_force[0]);
*/

  printw("actuator joint enable    [ %10.3f ]\n", actuator.joint_enable[0]);
  printw("actuator joint mode      [ %10.3f ]\n", actuator.joint_mode[0]);
  printw("actuator joint position  [ %10.3f ]\n", actuator.joint_position[0]);
  printw("actuator joint force     [ %10.3f ]\n", actuator.joint_force[0]);
  printw("actuator joint stiffness [ %10.3f ]\n", actuator.joint_stiffness[0]);
  printw("actuator joint damping   [ %10.3f ]\n\n", actuator.joint_damping[0]);

  printw("sensor joint position    [ %10.3f ]\n", sensor.joint_position[0]);
  printw("sensor joint force       [ %10.3f ]\n", sensor.joint_force[0]);
  printw("sensor motor position    [ %10.3f ]\n", sensor.motor_position[0]);
  printw("sensor motor force       [ %10.3f ]\n", sensor.motor_force[0]);
  printw("sensor motor current     [ %10.3f ]\n", sensor.motor_current[0]);
  printw("sensor motor temperature [ %10.3f ]\n\n", sensor.motor_temperature[0]);

  printw("actuator thread fps      [ %10.3f ]\n", actuator_thread.get_fps());
  if (strlen(actuator_thread.get_error_message()))
  {
    printw("actuator thread error    [ %s ]\n", actuator_thread.get_error_message());
  }
  printw("\npress <q> to quit...");

}
