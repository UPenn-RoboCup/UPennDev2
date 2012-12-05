#include <math.h>
#include <vector>
#include <stdio.h>
#include <assert.h>
#include <unistd.h>
#include <pthread.h>
#include <string.h>
#include "config.h"
#include "utils.h"
#include "dcm.h"
#include "dynamixel_thread.h"

extern Dcm dcm;
extern Config config;

dynamixel_thread::dynamixel_thread()
{
  m_id.resize(0);
  sprintf(m_interface, "\0");
}

void dynamixel_thread::set_joints(std::vector<int> id)
{
  m_id = id;
}

void dynamixel_thread::set_interface(const char *interface)
{
  strncpy(m_interface, interface, 128);
}

void dynamixel_thread::entry()
{

}

void dynamixel_thread::update()
{

}

void dynamixel_thread::exit()
{

}
