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
#include "sensor_thread.h"
#include "utils.h"

// comms_manager : communications manager for actuator teststand
// author: Mike Hopkins
///////////////////////////////////////////////////////////////////////////

static void zero_joints();
static void draw_screen();
static void print_joint_array(double *joint_array);

using namespace shared_data;

namespace shared_data {
  extern struct actuator_data actuator;
  extern struct sensor_data sensor;
  extern struct bias_data bias;
};

namespace {
  epos_thread l_leg_thread;
  epos_thread r_leg_thread;
  sensor_thread _sensor_thread;
};

int main(int argc, char **argv)
{
  // initialize shared data
  shared_data::entry();

  // initialize communication threads
  std::vector<int> l_leg_joints(6);
  std::vector<int> r_leg_joints(6);
  for (int i = 0; i < 6; i++)
  {
    l_leg_joints[i] = i;
    r_leg_joints[i] = i + 6;
  }
  l_leg_thread.set_can_interface("can1");
  r_leg_thread.set_can_interface("can0");
  l_leg_thread.set_joints(l_leg_joints);
  r_leg_thread.set_joints(r_leg_joints);
  //_sensor_thread.set_can_interface("can2");
  //_sensor_thread.set_imu_interface("/dev/ttyACM0");

  // start communication threads
  fprintf(stderr, "starting comms threads...\n");
  l_leg_thread.start();
  r_leg_thread.start();
  while (!l_leg_thread.is_running() || !r_leg_thread.is_running()) {
    usleep(2e5);
  }
  //_sensor_thread.start();
  //while (!_sensor_thread.is_running()) {
  //  usleep(2e5);
  //}

  // move joints to zero position
  fprintf(stderr, "zeroing joints...\n");
  usleep(2e5);
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
    if (!l_leg_thread.is_running())
       break;
    if (!r_leg_thread.is_running())
       break;
//  if (!_sensor_thread.is_running())
//     break;
  }
  endwin();
  
  // stop communication threads 
  l_leg_thread.stop();
  r_leg_thread.stop();
  //_sensor_thread.stop();

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
  printw("                         ASH Communications Manager\n");
  printw("///////////////////////////////////////");
  printw("///////////////////////////////////////\n\n");
  
  printw("enable\n");
  print_joint_array(actuator.joint_enable);
  printw("mode\n");
  print_joint_array(actuator.joint_mode);
  printw("position\n");
  print_joint_array(sensor.joint_position);
  printw("force\n");
  print_joint_array(sensor.joint_force);
  printw("stiffness\n");
  print_joint_array(actuator.joint_stiffness);
  printw("damping\n");
  print_joint_array(actuator.joint_damping);
  printw("thread fps\n");
  printw("            [ %7.0f %7.0f   ]\n", 
    l_leg_thread.get_fps(), r_leg_thread.get_fps());
  if (strlen(l_leg_thread.get_error_message()))
  {
    printw("l_leg error\n");
    printw("            [ %s ]\n", l_leg_thread.get_error_message());
  }
  if (strlen(r_leg_thread.get_error_message()))
  {
    printw("r_leg error\n");
    printw("            [ %s ]\n", r_leg_thread.get_error_message());
  }
  printw("\npress <q> to quit...");
}

void print_joint_array(double *joint_array)
{
  printw("            [ ");
  for (int i = 0; i < 6; i++)
    printw("%7.3f ", joint_array[i]);
  printw("  ]   l_leg\n");
  printw("            [ ");
  for (int i = 6; i < 12; i++)
    printw("%7.3f ", joint_array[i]);
  printw("  ]   r_leg\n");
}

