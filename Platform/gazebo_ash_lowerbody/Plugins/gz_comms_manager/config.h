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
#define P_GAIN_CONSTANT 1500
#define I_GAIN_CONSTANT 1500
#define D_GAIN_CONSTANT 1500
#define D_BREAK_FREQUENCY 50

// define initial controller gains
#define P_GAIN_DEFAULT 0.8
#define I_GAIN_DEFAULT 0.0
#define D_GAIN_DEFAULT 0.01

#endif
