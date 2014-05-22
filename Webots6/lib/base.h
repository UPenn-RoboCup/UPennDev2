/*
 * File:          base.h
 * Date:          24th May 2011
 * Description:   Allows to handle the base
 * Author:        fabien.rohrer@cyberbotics.com
 * Modifications: 
 */

#ifndef BASE_H
#define BASE_H

#include <webots/types.h>

void base_init();

void base_reset();
void base_forwards();
void base_backwards();
void base_turn_left();
void base_turn_right();
void base_strafe_left();
void base_strafe_right();

void base_goto_init(double time_step);
void base_goto_set_target(double x, double z, double a);
void base_goto_run();
bool base_goto_reached();

#endif
