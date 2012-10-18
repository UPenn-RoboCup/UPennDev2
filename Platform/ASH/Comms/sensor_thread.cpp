#include <math.h>
#include <string.h>
#include "sensor_thread.h"
#include "dcm.h"
#include "config.h"

// sensor_thread : motion sensor communication thread for ASH
// author : Mike Hopkins
///////////////////////////////////////////////////////////////////////////

extern Dcm dcm;
extern Config config;

sensor_thread::sensor_thread()
{
  sprintf(m_can_interface, "\0");
  sprintf(m_imu_interface, "\0");
}

void sensor_thread::set_can_interface(const char *interface)
{
  strncpy(m_can_interface, interface, 128);
}

void sensor_thread::set_imu_interface(const char *interface)
{
  strncpy(m_imu_interface, interface, 128);
}

void sensor_thread::entry()
{
  double gyro_bias[3] = {0, 0, 0};
  m_imu.openPort(m_imu_interface);
  m_imu.initTime(0);
//m_imu.initGyros(&gyro_bias[0], &gyro_bias[1], &gyro_bias[2]);
}

void sensor_thread::update()
{
    // update imu data
    uint64_t time;
    double accel[3], gyro[3], orientation[9];
    uint8_t cmd[] = {Microstrain::CMD_ACCEL_ANGRATE_ORIENT};
    m_imu.send(cmd, sizeof(cmd));
    m_imu.receiveAccelAngrateOrientation(&time, accel, gyro, orientation);

    // upate sensor shared memory
    dcm.ahrs[0] = accel[1];                              // x accel
    dcm.ahrs[1] = accel[0];                              // y accel
    dcm.ahrs[2] = -accel[2];                             // z accel
    dcm.ahrs[3] = gyro[1];                               // x gyro
    dcm.ahrs[4] = gyro[0];                               // y gyro
    dcm.ahrs[5] = -gyro[2];                              // z gyro
    dcm.ahrs[7] = -asin(orientation[2]);                 // x euler
    dcm.ahrs[6] = atan(orientation[5]/orientation[8]);   // y euler
    dcm.ahrs[8] = -atan2(orientation[1],orientation[0]); // z euler
}

void sensor_thread::exit()
{
  m_imu.closePort();
}
