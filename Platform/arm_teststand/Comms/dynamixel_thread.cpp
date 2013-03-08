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

    //printf("Writing successful!\n");
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

void dynamixel_thread::home_motor_controllers()
{
  // send motor to home
  //Write(Port, 1, 596, (long long)130000, 4);
}

void dynamixel_thread::update_actuator_settings()
{
  
  
  unsigned int value = 0;
  int error = 0;
  double ivalue;
  int i;
  
  for(i=0; i<12; i++)
  {
    if(dcm.joint_enable_updated[i])
    {
      dcm.joint_enable_updated[i] = 0;
      //if(dcm.joint_enable)
      Write(Port, i+3, 562, dcm.joint_enable[i], 1);
    }
  }
  
  for(i=12; i<19; i++)
  {
    if(dcm.joint_enable_updated[i])
    {
      dcm.joint_enable_updated[i] = 0;
      //if(dcm.joint_enable)
      Write(Port, i+3, 24, dcm.joint_enable[i], 1);
    }
  }
  
  //////////////////////////////////////////////
  //  Left arm
 
  // Update from DCM
  // 
  ivalue = dcm.joint_position[0] * 79592.0;
  value = (unsigned int)ivalue;
  //if(abs(value) > 100000)
  //  value = 0;
  Write(Port, 4, 596, (long long)value, 4);
  
  ivalue = dcm.joint_position[1] * 79592.0;
  value = (unsigned int)ivalue;
  //if(abs(value) > 100000)
  //  value = 0;
  Write(Port, 6, 596, (long long)value, 4);
  
  ivalue = dcm.joint_position[2] * 79592.0;
  value = (unsigned int)ivalue;
  //if(abs(value) > 100000)
  //  value = 0;
  Write(Port, 8, 596, (long long)value, 4);
  
  ivalue = dcm.joint_position[3] * 79592.0;
  value = (unsigned int)ivalue;
  //if(abs(value) > 100000)
  //  value = 0;
  Write(Port, 10, 596, (long long)value, 4);
  
  ivalue = dcm.joint_position[4] * 79592.0;
  value = (unsigned int)ivalue;
  //if(abs(value) > 100000)
  //  value = 0;
  Write(Port, 12, 596, (long long)value, 4);
  
  ivalue = dcm.joint_position[5] * 79592.0;
  value = (unsigned int)ivalue;
  //if(abs(value) > 100000)
  //  value = 0;
  Write(Port, 14, 596, (long long)value, 4);
  
  
  ////////
  // Fingers
  
  ivalue = dcm.joint_position[12] * 1129.0;
  value = (int)ivalue;
  Write(Port, 16, 30, value, 2);    // low byte
  
  //value = (int)ivalue;
  //Write(Port, 7, 31, value / 256, 1);   // high byte
  
  ivalue = dcm.joint_position[13] * 1129.0;
  value = (int)ivalue;
  Write(Port, 18, 30, value, 2);
  //value = (int)ivalue;
  //Write(Port, 8, 31, value / 256, 1);
  
  ivalue = dcm.joint_position[14] * 1129.0;
  value = (int)ivalue;
  Write(Port, 20, 30, value, 2);
  //value = (int)ivalue;
  //Write(Port, 9, 31, value / 256, 1);
 
  // --------------------------- Other arm
 
  // Update from DCM
  ivalue = dcm.joint_position[6] * 79592.0;
  value = (unsigned int)ivalue;
  //if(abs(value) > 100000)
  //  value = 0;
  Write(Port, 3, 596, (long long)value, 4);
  
  ivalue = dcm.joint_position[7] * 79592.0;
  value = (unsigned int)ivalue;
  //if(abs(value) > 100000)
  //  value = 0;
  Write(Port, 5, 596, (long long)value, 4);
  
  ivalue = dcm.joint_position[8] * 79592.0;
  value = (unsigned int)ivalue;
  //if(abs(value) > 100000)
  //  value = 0;
  Write(Port, 7, 596, (long long)value, 4);
  
  ivalue = dcm.joint_position[9] * 79592.0;
  value = (unsigned int)ivalue;
  //if(abs(value) > 100000)
  //  value = 0;
  Write(Port, 9, 596, (long long)value, 4);
  
  ivalue = dcm.joint_position[10] * 79592.0;
  value = (unsigned int)ivalue;
  //if(abs(value) > 100000)
  //  value = 0;
  Write(Port, 11, 596, (long long)value, 4);
  
  ivalue = dcm.joint_position[11] * 79592.0;
  value = (unsigned int)ivalue;
  //if(abs(value) > 100000)
  //  value = 0;
  Write(Port, 13, 596, (long long)value, 4);
  
  
  ////////
  // Fingers
  
  ivalue = dcm.joint_position[15] * 1129.0;
  value = (int)ivalue;
  Write(Port, 15, 30, value, 2);    // low byte
  
  //value = (int)ivalue;
  //Write(Port, 7, 31, value / 256, 1);   // high byte
  
  ivalue = dcm.joint_position[16] * 1129.0;
  value = (int)ivalue;
  Write(Port, 17, 30, value, 2);
  //value = (int)ivalue;
  //Write(Port, 8, 31, value / 256, 1);
  
  ivalue = dcm.joint_position[17] * 1129.0;
  value = (int)ivalue;
  Write(Port, 19, 30, value, 2);
  //value = (int)ivalue;
  //Write(Port, 9, 31, value / 256, 1);
}

void dynamixel_thread::update_sensor_readings()
{
  // Poll motor position
  unsigned int value = 0;
  int low = 0;
  int high = 0;
  int error = 0;
  double ivalue = 0;
 
  // Update in DCM
  // left shoulder pitch
  dxl_read_dword(Port, 4, 611, &value, &error);
  ivalue = (int)value;
  //ivalue = ivalue/79592;
  dcm.joint_position_sensor[0] = (double)ivalue/79592.0;
  
  // left shoulder roll
  dxl_read_dword(Port, 6, 611, &value, &error);
  ivalue = (int)value;
  //ivalue = ivalue/79592;
  dcm.joint_position_sensor[1] = (double)ivalue/79592.0;
  
  //left shoulder yaw
  dxl_read_dword(Port, 8, 611, &value, &error);
  ivalue = (int)value;
  //ivalue = ivalue/79592;
  dcm.joint_position_sensor[2] = (double)ivalue/79592.0;
  
  // left elbow pitch
  dxl_read_dword(Port, 10, 611, &value, &error);
  ivalue = (int)value;
  //ivalue = ivalue/79592;
  dcm.joint_position_sensor[3] = (double)ivalue/79592.0;
  
  // left wrist yaw
  dxl_read_dword(Port, 12, 611, &value, &error);
  ivalue = (int)value;
  //ivalue = ivalue/79592;
  dcm.joint_position_sensor[4] = (double)ivalue/79592.0;
  
  // left wrist roll
  dxl_read_dword(Port, 14, 611, &value, &error);
  ivalue = (int)value;
  //ivalue = ivalue/79592;
  dcm.joint_position_sensor[5] = (double)ivalue/79592.0;
  
  // Right arm
  // Right shoulder pitch
  dxl_read_dword(Port, 3, 611, &value, &error);
  ivalue = (int)value;
  //ivalue = ivalue/79592;
  dcm.joint_position_sensor[6] = (double)ivalue/79592.0;
  
  // Right shoulder roll 
  dxl_read_dword(Port, 5, 611, &value, &error);
  ivalue = (int)value;
  //ivalue = ivalue/79592;
  dcm.joint_position_sensor[7] = (double)ivalue/79592.0;
  
  // right shoulder yaw
  dxl_read_dword(Port, 7, 611, &value, &error);
  ivalue = (int)value;
  //ivalue = ivalue/79592;
  dcm.joint_position_sensor[8] = (double)ivalue/79592.0;
  
  // right elbow pitch
  dxl_read_dword(Port, 9, 611, &value, &error);
  ivalue = (int)value;
  //ivalue = ivalue/79592;
  dcm.joint_position_sensor[9] = (double)ivalue/79592.0;
  
  // right wrist yaw
  dxl_read_dword(Port, 11, 611, &value, &error);
  ivalue = (int)value;
  //ivalue = ivalue/79592;
  dcm.joint_position_sensor[10] = (double)ivalue/79592.0;
  
  // right wrist roll
  dxl_read_dword(Port, 13, 611, &value, &error);
  ivalue = (int)value;
  //ivalue = ivalue/79592;
  dcm.joint_position_sensor[11] = (double)ivalue/79592.0;
  
  ////////////////
  // Read MX fingers
  
  // left thumb
  dxl_read_word(Port, 16, 36, &low, &error);
  //dxl_read_byte(Port, 7, 37, &high, &error);
  //ivalue = (int)low + (int)high * 256;
  ivalue = (int)low;
  //ivalue = ivalue/79592;
  dcm.joint_position_sensor[12] = (double)ivalue/1129.0;
  
  // left finger 1
  dxl_read_word(Port, 18, 36, &low, &error);
  //dxl_read_byte(Port, 8, 37, &high, &error);
  //ivalue = (int)low + (int)high * 256;
  ivalue = (int)low;
  //ivalue = ivalue/79592;
  dcm.joint_position_sensor[13] = (double)ivalue/1129.0;
  
  // left finger 2
  dxl_read_word(Port, 20, 36, &low, &error);
  //dxl_read_byte(Port, 9, 37, &high, &error);
  //ivalue = (int)low + (int)high * 256;
  ivalue = (int)low;
  //ivalue = ivalue/79592;
  dcm.joint_position_sensor[14] = (double)ivalue/1129.0;
  
  // right thumb
  dxl_read_word(Port, 15, 36, &low, &error);
  //dxl_read_byte(Port, 7, 37, &high, &error);
  //ivalue = (int)low + (int)high * 256;
  ivalue = (int)low;
  //ivalue = ivalue/79592;
  dcm.joint_position_sensor[15] = (double)ivalue/1129.0;
  
  // right Finger 1
  dxl_read_word(Port, 17, 36, &low, &error);
  //dxl_read_byte(Port, 8, 37, &high, &error);
  //ivalue = (int)low + (int)high * 256;
  ivalue = (int)low;
  //ivalue = ivalue/79592;
  dcm.joint_position_sensor[16] = (double)ivalue/1129.0;
  
  // right finger 2
  dxl_read_word(Port, 19, 36, &low, &error);
  //dxl_read_byte(Port, 9, 37, &high, &error);
  //ivalue = (int)low + (int)high * 256;
  ivalue = (int)low;
  //ivalue = ivalue/79592;
  dcm.joint_position_sensor[17] = (double)ivalue/1129.0;
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
  
  /*
  // Torque on motor
  Write(Port, 1, 562, 1, 1);
  Write(Port, 2, 562, 1, 1);
  Write(Port, 3, 562, 1, 1);
  Write(Port, 4, 562, 1, 1);
  Write(Port, 5, 562, 1, 1);
  Write(Port, 6, 562, 1, 1);
  
  // finger torque on
  Write(Port, 7, 24, 1, 1);
  Write(Port, 8, 24, 1, 1);
  Write(Port, 9, 24, 1, 1);
  */
  
  
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
  
  // Send each motor back to zero position
  //home_motor_controllers();
  
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




