#include <math.h>
#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <unistd.h>
#include <pthread.h>
#include <string.h>

#include "config.h"
#include "utils.h"
#include "dcm.h"
#include "dynamixel_thread.h"
#include "dynamixel_settings.h"

extern Dcm dcm;
extern Config config;

// Print error bit of status packet
void PrintErrorCode(int ErrorCode)
{
    if(ErrorCode & ERRBIT_VOLTAGE)
        printf("Input voltage error!\n");

    if(ErrorCode & ERRBIT_ANGLE)
        printf("Angle limit error!\n");

    if(ErrorCode & ERRBIT_OVERHEAT)
        printf("Overheat error!\n");

    if(ErrorCode & ERRBIT_RANGE)
        printf("Out of range error!\n");

    if(ErrorCode & ERRBIT_CHECKSUM)
        printf("Checksum error!\n");

    if(ErrorCode & ERRBIT_OVERLOAD)
        printf("Overload error!\n");

    if(ErrorCode & ERRBIT_INSTRUCTION)
        printf("Instruction code error!\n");
}

void Write(PSerialPort port, int id, int addr, unsigned int value, int len)
{
    int result = COMM_TXFAIL, error = 0;

    if(len == 1)
        result = dxl_write_byte(port, id, addr, (int)value, &error);
    else if(len == 2)
        result = dxl_write_word(port, id, addr, (int)value, &error);
    else if(len == 4)
        result = dxl_write_dword(port, id, addr, value, &error);

    if(result != COMM_RXSUCCESS)
    {
        printf("Fail to write!\n");
        return;
    }

    if(error != 0)
    {
        PrintErrorCode(error);
        return;
    }
}

void Read(PSerialPort port, int id, int addr, int len)
{
    int result = COMM_TXFAIL, error = 0;
    unsigned int value = 0;
    int ivalue = 0;

    if(len == 1)
        result = dxl_read_byte(port, id, addr, &ivalue, &error);
    else if(len == 2)
        result = dxl_read_word(port, id, addr, &ivalue, &error);
    else if(len == 4)
        result = dxl_read_dword(port, id, addr, &value, &error);

    if(result != COMM_RXSUCCESS)
    {
        printf("Fail to read!\n");
        return;
    }

    if(error != 0)
    {
        PrintErrorCode(error);
        return;
    }

    if(len == 1 || len == 2)
        value = (unsigned int)ivalue;

    printf("Read value : %d\n", value);
}

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

void dynamixel_thread::update_actuator_settings()
{
  unsigned int value = 0;
  int error = 0;
  double ivalue;
  int i;
 
  //update torque values from DCM 
  for(i = 0; i < N_JOINT; i++)
  {
    if(dcm.joint_enable_updated[i])
    {
      dcm.joint_enable_updated[i] = 0;
      if(dynamixel_series[i] == NX_DXL)
      {
        Write(Port, dynamixel_id[i], 562, dcm.joint_enable[i], 1);
      }
      else
      {
        Write(Port, dynamixel_id[i], 24, dcm.joint_enable[i], 1);  
      }
    }
  }
  
  //update position values from DCM
  for(i = 0; i < N_JOINT; i++)
  {
    if(dynamixel_series[i] == NX_DXL)
    {
      ivalue = dcm.joint_position[i] * 79592.0;
      value = (unsigned int)ivalue;
      Write(Port, dynamixel_id[i], 596, (long long)value, 4);
    }
    else
    {
      ivalue = dcm.joint_position[i] * 1129.0;
      value = (int)ivalue;
      Write(Port, dynamixel_id[i], 30, value, 2); 
    }
  }
}

void dynamixel_thread::update_sensor_readings()
{
  // Poll motor position
  unsigned int value = 0;
  int low = 0;
  int high = 0;
  int error = 0;
  double ivalue = 0;
  int i;
 
  // Update DCM position values
  for(i = 0; i < N_JOINT; i++)
  {
    if(dynamixel_series[i] == NX_DXL)
    {
      dxl_read_dword(Port, dynamixel_id[i], 611, &value, &error);
      ivalue = (int)value;
      dcm.joint_position_sensor[i] = (double)ivalue/79592.0;
    }
    else
    {
      dxl_read_word(Port, dynamixel_id[i], 36, &low, &error);
      ivalue = (int)low;
      dcm.joint_position_sensor[i] = (double)ivalue/1129.0;
    }
  }
}

void dynamixel_thread::entry()
{

  Port = &sp;
  int port_num = 0, baudnum = 1;
  int i;
  
  // Initialize serial interface
  if(dxl_initialize(Port, port_num, baudnum) == 0)
    {
        printf( "Failed to open USB2Dynamixel!\n" );
        printf( "Press any key to terminate...\n" );
        return;
    }
    else
        printf( "Succeed to open USB2Dynamixel!\n\n" );
  
  // Torque off motors on startup
  for(i = 0; i < N_JOINT; i++)
  {
    if(dynamixel_series[i] == NX_DXL)
    {
      Write(Port, dynamixel_id[i], 562, 0, 1);
    }
    else
    {
      Write(Port, dynamixel_id[i], 24, 0, 1);
    }
  }

  update_sensor_readings();
 
  // Ensure motors stay in their initial positions
  for(i = 0; i < N_JOINT; i++)
  {
    dcm.joint_position[i] = dcm.joint_position_sensor[i];
  }
}

void dynamixel_thread::update()
{
  
  update_actuator_settings();
  
  update_sensor_readings();
  
}

void dynamixel_thread::exit()
{
  int i;

  // Torque off all motors
  for(i = 0; i < N_JOINT; i++)
  {
    if(dynamixel_series[i] == NX_DXL)
    {
      Write(Port, dynamixel_id[i], 562, 0, 1);
    }
    else
    {
      Write(Port, dynamixel_id[i], 24, 0, 1);
    }
  }

  // Close serial port
  if(Port != NULL)
    dxl_terminate(Port);
}




