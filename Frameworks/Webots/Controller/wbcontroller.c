#include <webots/robot.h>
#include <webots/camera.h>
#include <webots/servo.h>
#include <webots/gyro.h>
#include <webots/accelerometer.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <string.h>

#include <zmq.h>

void *ctx, *csocket;

/* DarwinOP names in webots */
const int nJoint = 20;
const char *jointNames[] = {"Neck", "Head",
              "ShoulderL", "ArmUpperL", "ArmLowerL",
              "PelvYL", "PelvL", "LegUpperL", "LegLowerL", "AnkleL", "FootL", 
              "PelvYR", "PelvR", "LegUpperR", "LegLowerR", "AnkleR", "FootR",
              "ShoulderR", "ArmUpperR", "ArmLowerR"};

const double jointBias[] = {	0,0,
            	M_PI/2,0,M_PI/2,
            	0,0,0,0,0,0,
            	0,0,0,0,0,0,
            	M_PI/2,0,M_PI/2,
};

const int moveDir[] = {-1, 1, 
                    1, 1, 1,
                    1, -1, -1, -1, 1, 1,
                    1, 1, 1, 1, -1, 1,
                    -1, 1, -1};

const int indexHead = 1;			/* Head: 1 2 */
const int nJointHead = 2;
const int indexLArm = 3;			/* LArm: 3 4 5 */
const int nJointLArm = 3; 		
const int indexLLeg = 6;			/* LLeg:6 7 8 9 10 11 */
const int nJointLLeg = 6;
const int indexRLeg = 12; 		/* RLeg: 12 13 14 15 16 17 */
const int nJointRLeg = 6;
const int indexRArm = 18; 		/* RArm: 18 19 20 */
const int nJointRArm = 3;

//double actuator_position[20];

struct wb_devices {
  WbDeviceTag camera;
  WbDeviceTag accelerometer;
  WbDeviceTag gyro;
  WbDeviceTag gps;
  WbDeviceTag compass;
  WbDeviceTag eyeled;
  WbDeviceTag headled;
  WbDeviceTag* joints;
};

int get_image(const unsigned char * raw, unsigned char *rgb,
                int width, int height) {
  int x, y, r, g, b, rgb_index = 0;
  for (x = 0; x < width; x++)
    for (y = 0; y < height; y++) {
      rgb[rgb_index++] = wb_camera_image_get_red(raw, width, x, y);
      rgb[rgb_index++] = wb_camera_image_get_green(raw, width, x, y);
      rgb[rgb_index++] = wb_camera_image_get_blue(raw, width, x, y);
    }
  return 0;
}

int main() {
  struct wb_devices tags;
  /* init webots robot */
  wb_robot_init();
  /* get basic time step */
  double timeStep = wb_robot_get_basic_time_step();

  /* init actuator */
  tags.joints = malloc(nJoint * sizeof(WbDeviceTag));  
  double *actuator_position = malloc(nJoint * sizeof(double));

  int i = 0;
  for (i = 0; i < nJoint; i++) {
    tags.joints[i] = wb_robot_get_device(jointNames[i]);
    wb_servo_enable_position(tags.joints[i], timeStep);
    printf("init Joint %s\n", jointNames[i]);
  }

  /* init camera */
  tags.camera = wb_robot_get_device("Camera");
  wb_camera_enable(tags.camera, timeStep);

  int height = wb_camera_get_height(tags.camera);
  int width = wb_camera_get_width(tags.camera);
  printf("Camera width: %d height: %d\n", width, height);
  const unsigned char * raw_img = NULL;
  unsigned char rgb_img[width * height * 3];

  /* init acc and gyro */
  tags.accelerometer = wb_robot_get_device("Accelerometer");
  wb_accelerometer_enable(tags.accelerometer, timeStep);
  tags.gyro = wb_robot_get_device("Gyro");
  wb_gyro_enable(tags.gyro, timeStep);

  /* init leds */
  tags.eyeled = wb_robot_get_device("EyeLed");
  wb_led_set(tags.eyeled,0xffffff);
  tags.headled = wb_robot_get_device("HeadLed");
  wb_led_set(tags.headled,0x00ff00);

  /* init zmq */
  ctx = zmq_init(1);
  assert(ctx);
  csocket = zmq_socket(ctx, ZMQ_PUB);
  assert(csocket);
  int rc2 = zmq_bind(csocket, "ipc:///tmp/test");
  assert(rc2==0);

  wb_robot_step(timeStep);
  int x, y, r, g, b;
  double accel[3];
  double gyro[3];
  while(1) {
    /* atuator update */
    double t;
    for (i = 0; i < nJoint; i++) {
      actuator_position[i] = moveDir[i] * wb_servo_get_position(tags.joints[i])-jointBias[i];
    }
    printf("joints %f %f %f\n", actuator_position[0], actuator_position[1], actuator_position[3]);

    /* imu update */
    memcpy(accel, wb_accelerometer_get_values(tags.accelerometer), 3 * sizeof(double));
    memcpy(gyro, wb_gyro_get_values(tags.gyro), 3 * sizeof(double));
    printf("acc %f %f %f, gyro %f %f %f\n", accel[0], accel[1], accel[2], gyro[0], gyro[1], gyro[2]);

    /* image update */
    raw_img = wb_camera_get_image(tags.camera);
    get_image(raw_img, rgb_img, width, height);
    printf("%d %d %d\n", rgb_img[0], rgb_img[1], rgb_img[2]);
    zmq_send(csocket, (void *)rgb_img, width*height*3, 0);

    wb_robot_step(timeStep);
  }
  free(tags.joints);
  free(actuator_position);
  return 0;
}
