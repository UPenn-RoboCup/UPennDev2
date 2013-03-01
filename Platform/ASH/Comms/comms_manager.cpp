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
#include "epos_thread.h"
#include "sensor_thread.h"
#include "utils.h"

// comms_manager : communications manager for ash
// author : Mike Hopkins
///////////////////////////////////////////////////////////////////////////

extern Dcm dcm;
extern Config config;
static epos_thread l_leg_thread;
static epos_thread r_leg_thread;
static sensor_thread motion_sensor_thread;

static void zero_joints();
static void draw_screen();
static void print_force_torque(double *force_torque);
static void print_joint_array(double *joint_array);

int main(int argc, char **argv)
{
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
  motion_sensor_thread.set_can_interface("can2");
  motion_sensor_thread.set_imu_interface("/dev/ttyACM0");

  // start communication threads
  fprintf(stderr, "starting comms threads...\n");
  motion_sensor_thread.start();
  while (!motion_sensor_thread.is_running()) {
    usleep(2e5);
  }
  l_leg_thread.start();
  r_leg_thread.start();
  while (!l_leg_thread.is_running() || !r_leg_thread.is_running()) {
    usleep(2e5);
  }

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
    if (!motion_sensor_thread.is_running())
       break;
  }
  draw_screen();
  refresh();
//endwin();
  
  // stop communication threads 
  l_leg_thread.stop();
  r_leg_thread.stop();
  motion_sensor_thread.stop();

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
  printw("                         ASH Communications Manager\n");
  printw("///////////////////////////////////////");
  printw("///////////////////////////////////////\n\n");
  
  printw("enable\n");
  print_joint_array(dcm.joint_enable);
  printw("stiffness\n");
  print_joint_array(dcm.joint_stiffness);
  printw("force\n");
  print_joint_array(dcm.joint_force_sensor);
  printw("position\n");
  print_joint_array(dcm.joint_position_sensor);
  printw("velocity\n");
  print_joint_array(dcm.joint_velocity_sensor);
  printw("force torque\n");
  print_force_torque(dcm.force_torque);
  printw("thread fps\n");
  printw("            [ %7.0f %7.0f %7.0f ]\n", 
    l_leg_thread.get_fps(), r_leg_thread.get_fps(), motion_sensor_thread.get_fps());
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
  if (strlen(motion_sensor_thread.get_error_message()))
  {
    printw("sensor error\n");
    printw("            [ %s ]\n", motion_sensor_thread.get_error_message());
  }
  printw("\npress <q> to quit...");
}

void print_force_torque(double *force_torque)
{
  printw("            [ ");
  for (int i = 0; i < 6; i++)
    printw("%7.3f ", force_torque[i]);
  printw("  ]\n");
  printw("            [ ");
  for (int i = 6; i < 12; i++)
    printw("%7.3f ", force_torque[i]);
  printw("  ]\n");
}

void print_joint_array(double *joint_array)
{
  printw("            [ ");
  for (int i = 0; i < 6; i++)
    printw("%7.3f ", joint_array[i]);
  printw("  ]\n");
  printw("            [ ");
  for (int i = 6; i < 12; i++)
    printw("%7.3f ", joint_array[i]);
  printw("  ]\n");
}
