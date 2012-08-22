#include <unistd.h>
#include <stdlib.h>
#include <math.h>
#include "Microstrain.h"

int main()
{
  const char *port_name = "/dev/ttyACM0";

  /* initialize imu */
  Microstrain imu;
  double gyro_bias[3] = {0, 0, 0};
  imu.openPort(port_name);
  imu.initTime(0);
  //imu.initGyros(gyro_bias, gyro_bias + 1, gyro_bias + 2);

  /* begin polling */
  while (1)
  {
    /* send request */
    uint8_t cmd[] = {Microstrain::CMD_ACCEL_ANGRATE_ORIENT};
    imu.send(cmd, sizeof(cmd));
    /* receive data */
    uint64_t time;
    double accel[3], gyro[3], orientation[9];
    imu.receiveAccelAngrateOrientation(&time, accel, gyro, orientation);
    fprintf(stderr, "x accel : %f\n", accel[1]);                          // x accel
    fprintf(stderr, "y accel : %f\n", accel[0]);                          // y accel
    fprintf(stderr, "z accel : %f\n", -accel[2]);                         // z accel
    fprintf(stderr, "pitch gyro : %f\n", gyro[0]);                        // pitch gyro
    fprintf(stderr, "roll gyro : %f\n", gyro[1]);                         // roll gyro
    fprintf(stderr, "yaw gyro : %f\n", -gyro[2]);                         // yaw gyro
    fprintf(stderr, "pitch : %f\n", atan(orientation[5]/orientation[8])); // pitch
    fprintf(stderr, "roll : %f\n", -asin(orientation[2]));                // roll
    fprintf(stderr, "yaw : %f\n", -atan2(orientation[1],orientation[0])); // yaw
    usleep(50000);
    system("clear");
  }
  return 0;
}
