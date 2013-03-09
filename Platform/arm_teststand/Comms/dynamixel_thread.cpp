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
  for(i=0; i<12; i++)
  {
    if(dcm.joint_enable_updated[i])
    {
      dcm.joint_enable_updated[i] = 0;
      Write(Port, dynamixel_id[i], 562, dcm.joint_enable[i], 1);
    }
  }
  
  for(i=12; i<18; i++)
  {
    if(dcm.joint_enable_updated[i])
    {
      dcm.joint_enable_updated[i] = 0;
      Write(Port, dynamixel_id[i], 24, dcm.joint_enable[i], 1);
    }
  }

  //update arms from DCM
  for(i=0; i<12; i++)
  {
    ivalue = dcm.joint_position[i] * 79592.0;
    value = (unsigned int)ivalue;
    Write(Port, dynamixel_id[i], 596, (long long)value, 4);
  }

  //update fingers from DCM
  for(i=12; i<18; i++)
  {
    ivalue = dcm.joint_position[i] * 1129.0;
    value = (int)ivalue;
    Write(Port, dynamixel_id[i], 30, value, 2); 
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
 
  // Update in DCM
  for(i=0; i<12; i++)
  {
    dxl_read_dword(Port, dynamixel_id[i], 611, &value, &error);
    ivalue = (int)value;
    dcm.joint_position_sensor[i] = (double)ivalue/79592.0;
  }

  //update fingers from DCM
  for(i=12; i<18; i++)
  {
   dxl_read_word(Port, dynamixel_id[i], 36, &low, &error);
   ivalue = (int)low;
   dcm.joint_position_sensor[i] = (double)ivalue/1129.0;
  }
}

void dynamixel_thread::entry()
{

  Port = &sp;
  int port_num = 0, baudnum = 1;
  
  // Initialize serial interface
  if(dxl_initialize(Port, port_num, baudnum) == 0)
    {
        printf( "Failed to open USB2Dynamixel!\n" );
        printf( "Press any key to terminate...\n" );
        return;
    }
    else
        printf( "Succeed to open USB2Dynamixel!\n\n" );
  
  // Torque off motors
  Write(Port, 3, 562, 0, 1);
  Write(Port, 4, 562, 0, 1);
  Write(Port, 5, 562, 0, 1);
  Write(Port, 6, 562, 0, 1);
  Write(Port, 7, 562, 0, 1);
  Write(Port, 8, 562, 0, 1);
  Write(Port, 9, 562, 0, 1);
  Write(Port, 10, 562, 0, 1);
  Write(Port, 11, 562, 0, 1);
  Write(Port, 12, 562, 0, 1);
  Write(Port, 13, 562, 0, 1);
  Write(Port, 14, 562, 0, 1);
  
  // finger torque off
  Write(Port, 15, 24, 0, 1);
  Write(Port, 16, 24, 0, 1);
  Write(Port, 17, 24, 0, 1);
  Write(Port, 18, 24, 0, 1);
  Write(Port, 19, 24, 0, 1);
  Write(Port, 20, 24, 0, 1);

  update_sensor_readings();
 
  dcm.joint_position[0] = dcm.joint_position_sensor[0];
  dcm.joint_position[1] = dcm.joint_position_sensor[1];
  dcm.joint_position[2] = dcm.joint_position_sensor[2];
  dcm.joint_position[3] = dcm.joint_position_sensor[3];
  dcm.joint_position[4] = dcm.joint_position_sensor[4];
  dcm.joint_position[5] = dcm.joint_position_sensor[5];
  dcm.joint_position[6] = dcm.joint_position_sensor[6];
  dcm.joint_position[7] = dcm.joint_position_sensor[7];
  dcm.joint_position[8] = dcm.joint_position_sensor[8];
  dcm.joint_position[9] = dcm.joint_position_sensor[9];
  dcm.joint_position[10] = dcm.joint_position_sensor[10];
  dcm.joint_position[11] = dcm.joint_position_sensor[11];
  dcm.joint_position[12] = dcm.joint_position_sensor[12];
  dcm.joint_position[13] = dcm.joint_position_sensor[13];
  dcm.joint_position[14] = dcm.joint_position_sensor[14];
  dcm.joint_position[15] = dcm.joint_position_sensor[15];
  dcm.joint_position[16] = dcm.joint_position_sensor[16];
  dcm.joint_position[17] = dcm.joint_position_sensor[17];
  dcm.joint_position[18] = dcm.joint_position_sensor[18];
}

void dynamixel_thread::update()
{
  
  update_actuator_settings();
  
  update_sensor_readings();
  
}

void dynamixel_thread::exit()
{

  // Torque off motor
  Write(Port, 1, 562, 0, 1);
  Write(Port, 2, 562, 0, 1);
  Write(Port, 3, 562, 0, 1);
  Write(Port, 4, 562, 0, 1);
  Write(Port, 5, 562, 0, 1);
  Write(Port, 6, 562, 0, 1);
  Write(Port, 7, 562, 0, 1);
  Write(Port, 8, 562, 0, 1);
  Write(Port, 9, 562, 0, 1);
  Write(Port, 10, 562, 0, 1);
  Write(Port, 11, 562, 0, 1);
  Write(Port, 12, 562, 0, 1);
  
  // finger torque off
  Write(Port, 13, 24, 0, 1);
  Write(Port, 14, 24, 0, 1);
  Write(Port, 15, 24, 0, 1);
  Write(Port, 16, 24, 0, 1);
  Write(Port, 17, 24, 0, 1);
  Write(Port, 18, 24, 0, 1);

  // Close serial port
  if(Port != NULL)
    dxl_terminate(Port);
}




