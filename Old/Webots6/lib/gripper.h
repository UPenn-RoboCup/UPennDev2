/*
 * File:          gripper.h
 * Date:          24th May 2011
 * Description:   Allows to handle the gipper
 * Author:        fabien.rohrer@cyberbotics.com
 * Modifications: 
 */

#ifndef GRIPPER_H
#define GRIPPER_H

void gripper_init();

void gripper_grip(); // dangerous to grip an object with this function -> creates a lot of internal constraints
void gripper_release();
void gripper_set_gap(double gap);

#endif
