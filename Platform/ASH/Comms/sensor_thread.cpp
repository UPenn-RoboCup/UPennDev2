#include <math.h>
#include <string.h>
#include "shared_data.h"
#include "sensor_thread.h"

// sensor_thread : motion sensor communication thread for ASH
// author: Mike Hopkins
///////////////////////////////////////////////////////////////////////////

using namespace shared_data;

namespace shared_data {
  extern struct actuator_data actuator;
  extern struct sensor_data sensor;
  extern struct bias_data bias;
};

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
    sensor.ahrs[0] = accel[1];                              // x accel
    sensor.ahrs[1] = accel[0];                              // y accel
    sensor.ahrs[2] = -accel[2];                             // z accel
    sensor.ahrs[4] = gyro[1];                               // x gyro
    sensor.ahrs[3] = gyro[0];                               // y gyro
    sensor.ahrs[5] = -gyro[2];                              // z gyro
    sensor.ahrs[7] = -asin(orientation[2]);                 // x euler
    sensor.ahrs[6] = atan(orientation[5]/orientation[8]);   // y euler
    sensor.ahrs[8] = -atan2(orientation[1],orientation[0]); // z euler
}

void sensor_thread::exit()
{
  m_imu.closePort();
}
