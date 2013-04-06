#include <webots/robot.h>
#include <webots/camera.h>
#include <stdio.h>
#include <assert.h>

#include <zmq.h>

void *ctx, *csocket;

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
  /* init webots robot */
  wb_robot_init();
  /* get basic time step */
  double timeStep = wb_robot_get_basic_time_step();

  /* get camera */
  WbDeviceTag camera = wb_robot_get_device("Camera");
  wb_camera_enable(camera, timeStep);

  int height = wb_camera_get_height(camera);
  int width = wb_camera_get_width(camera);
  printf("Camera width: %d height: %d\n", width, height);
  const unsigned char * raw_img = NULL;
  unsigned char rgb_img[width * height * 3];

  /* init zmq */
  ctx = zmq_init(1);
  assert(ctx);
  csocket = zmq_socket(ctx, ZMQ_PUB);
  assert(csocket);
  int rc2 = zmq_bind(csocket, "ipc:///tmp/test");
  assert(rc2==0);

  wb_robot_step(timeStep);
  int x, y, r, g, b;
  while(1) {
    raw_img = wb_camera_get_image(camera);
    get_image(raw_img, rgb_img, width, height);
    printf("%d %d %d\n", rgb_img[0], rgb_img[1], rgb_img[2]);
    zmq_send(csocket, (void *)rgb_img, width*height*3, 0);

    wb_robot_step(timeStep);
  }
  return 0;
}
