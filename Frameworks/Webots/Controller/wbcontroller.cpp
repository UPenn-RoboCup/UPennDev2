#include <webots/robot.h>
#include <webots/camera.h>
#include <webots/servo.h>
#include <webots/gyro.h>
#include <webots/led.h>
#include <webots/accelerometer.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <string>
#include <vector>    
#include <iostream>    

#include "config.h"
#include <zmq.h>
#include <msgpack.hpp>

#define BUFLEN 8192

using namespace std;

struct wb_devices {
  WbDeviceTag camera;
  WbDeviceTag accelerometer;
  WbDeviceTag gyro;
  WbDeviceTag gps;
  WbDeviceTag compass;
  WbDeviceTag eyeled;
  WbDeviceTag headled;
  vector<WbDeviceTag> joints;
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
  // Load config
  Config config;

  vector<string> jointNames = config.get_string_vector("jointNames");
  int nJoint = jointNames.size();
  vector<double> jointBias = config.get_double_vector("jointBias");
  vector<int> moveDir = config.get_int_vector("moveDir");
  double vision_update_interval = 0.04;

  std::cout << "load Done " << endl;
  struct wb_devices tags;
  /* init webots robot */
  wb_robot_init();
  /* get basic time step */
  double timeStep = wb_robot_get_basic_time_step();

  /* init actuator */
  vector<double> actuator_position;
  for (int i = 0; i < nJoint; i++) {
    tags.joints.push_back(wb_robot_get_device(jointNames[i].c_str()));
    wb_servo_enable_position(tags.joints[i], timeStep);
    actuator_position.push_back(0);
    cout << "init Joint" << jointNames[i] << endl;
  }

  /* init camera */
  tags.camera = wb_robot_get_device("Camera");
  wb_camera_enable(tags.camera, timeStep);

  int height = wb_camera_get_height(tags.camera);
  int width = wb_camera_get_width(tags.camera);
  printf("Camera width: %d height: %d\n", width, height);
  const unsigned char * raw_img = wb_camera_get_image(tags.camera);

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
  void* ctx = zmq_init(4);
  assert(ctx);

  // Polling (for now, just on actuator commands)
  zmq_pollitem_t poll_items[1];

  void *camera_socket = zmq_socket(ctx, ZMQ_PUB);
  assert(camera_socket);
  int rc = zmq_bind(camera_socket, "ipc:///tmp/camera");
  assert(rc==0);

  void *imu_socket = zmq_socket(ctx, ZMQ_PUB);
  assert(imu_socket);
  rc = zmq_bind(imu_socket, "ipc:///tmp/imu");
  assert(rc==0);

  void *actuator_pub_socket = zmq_socket(ctx, ZMQ_PUB);
  assert(actuator_pub_socket);
  rc = zmq_bind(actuator_pub_socket, "ipc:///tmp/actuator");
  // when publishing, should always add "get" so that the filter works
  assert(rc==0);

  void *actuator_sub_socket = zmq_socket(ctx, ZMQ_SUB);
  // Listen to everything...
  zmq_setsockopt( actuator_sub_socket, ZMQ_SUBSCRIBE, "", 0 );
  // Should listen with a filter...
  //zmq_setsockopt( actuator_sub_socket, ZMQ_SUBSCRIBE, "set", 0 );
  assert(actuator_sub_socket);
  rc = zmq_connect(actuator_sub_socket, "ipc:///tmp/actuator_cmd");
  assert(rc==0);
  // Poll object
  poll_items[0].socket = actuator_sub_socket;
  poll_items[0].events = ZMQ_POLLIN;
  // Receiving buffer
  char motor_command_buf[BUFLEN];

  // init msgpack
  msgpack::sbuffer sbuf;

  wb_robot_step(timeStep);
  vector<double> imu;
  imu.resize(6);

  double last_vision_update_time = wb_robot_get_time();
  while(1) {
    int nBytes = zmq_recv(actuator_sub_socket, motor_command_buf, BUFLEN, ZMQ_DONTWAIT);
    printf("receive msg length %d\n", nBytes);
    if( nBytes>0 ){
      msgpack::unpacked msg;
      msgpack::unpack(&msg, motor_command_buf, nBytes);
      vector<double> new_actuator;
      new_actuator.resize(20);
      msg.get() >> new_actuator;
      for (int i = 0; i < 20; i++)
        cout << new_actuator[i] << ' ';
      cout << endl;
    }

    //rc = zmq_poll( poll_items, 1, 10 );
    //printf("%d events\n",rc);

    double t = wb_robot_get_time();
    /* Actuator update */
    for (int i = 0; i < nJoint; i++)
      actuator_position[i] = moveDir[i] * wb_servo_get_position(tags.joints[i])-jointBias[i];
    // Pack actuators
    sbuf.clear();
    msgpack::pack(sbuf, actuator_position);
    rc = zmq_send(actuator_pub_socket, (void *)sbuf.data(), sbuf.size(), 0);

    /* IMU update */
    const double * accel = wb_accelerometer_get_values(tags.accelerometer);
    copy(accel, accel + 3, imu.begin());
    const double * gyro = wb_gyro_get_values(tags.gyro);
    copy(gyro, gyro + 3, imu.begin()+3);
    sbuf.clear();
    msgpack::pack(sbuf, imu);
    rc = zmq_send(imu_socket, (void *)sbuf.data(), sbuf.size(), 0);

    /* image update */
    if ((t - last_vision_update_time) >= vision_update_interval) {
      last_vision_update_time = t;
      get_image(raw_img, rgb_img, width, height);
      rc = zmq_send(camera_socket, (void *)rgb_img, width*height*3, 0);
    }

    if (wb_robot_step(timeStep) < 0)
      exit(0);
    fflush(stdout);
  }

  return 0;
}
