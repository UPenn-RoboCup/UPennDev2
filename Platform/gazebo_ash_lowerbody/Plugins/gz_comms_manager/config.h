#ifndef _CONFIG_H_
#define _CONFIG_H_

#include <vector>

// config : gz_comms_manager parameters
///////////////////////////////////////////////////////////////////////////

// define device array lengths
#define N_JOINT 33
#define N_MOTOR 33
#define N_FORCE_TORQUE 24
#define N_AHRS 9 
#define N_BATTERY 3

// define impedance controller settings
#define POSITION_P_GAIN_CONSTANT 1500
#define POSITION_I_GAIN_CONSTANT 1500
#define VELOCITY_P_GAIN_CONSTANT 1500
#define VELOCITY_BREAK_FREQUENCY 50

// define initial controller gains
#define POSITION_P_GAIN_INIT 0.8
#define POSITION_I_GAIN_INIT 0.0
#define VELOCITY_P_GAIN_INIT 0.01

#endif
