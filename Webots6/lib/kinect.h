/*
 * File:          kinect.h
 * Date:          20th July 2011
 * Description:   Allows to handle the kinect
 * Author:        fabien.rohrer@cyberbotics.com
 * Modifications: 
 */

#ifndef KINECT_H
#define KINECT_H

#include <webots/types.h>

void kinect_init(double time_step);

bool kinect_is_available();

#endif
