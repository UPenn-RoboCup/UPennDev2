/*
 * File:          kinect.c
 * Date:          20th July 2011
 * Description:   Implement the functions defined in kinect.h
 * Author:        fabien.rohrer@cyberbotics.com
 * Modifications: 
 */

#include "kinect.h"

#include <webots/robot.h>
#include <webots/camera.h>

#include "tiny_math.h"

static WbDeviceTag kinect;

void kinect_init(double time_step) {
  kinect = wb_robot_get_device("kinect");
  
  if (kinect_is_available())
    wb_camera_enable(kinect, time_step);
}

bool kinect_is_available() {
  return (bool) kinect;
}
