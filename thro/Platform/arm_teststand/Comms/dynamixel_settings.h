#ifndef DYNAMIXEL_SETTINGS_H 
#define DYNAMIXEL_SETTINGS_H

#include "config.h"

//encoder tick conversion factors
#define NX_CONV 79592.0
#define MX_CONV 1129.0

//motor types
enum DXL_TYPE {
  MX_DXL,
  NX_DXL
};

//address values table, add entries as needed
struct ADDR_TYPE {
  int enable;
  int goal_pos;
  int cur_pos;
};

//NX series address values
const ADDR_TYPE nx_addr = {
  562,
  596,
  611
};

//MX series address values
const ADDR_TYPE mx_addr = {
  24,
  30,
  36
};

//mapping of dynamixel motor ids to DCM values
const int dynamixel_id[N_JOINT] = {
  4, 6, 8, 10, 12, 14,
  3, 5, 7, 9, 11, 13,
  16, 18, 20,
  15, 17, 19
};

//types corresponding to each DCM mapping
const DXL_TYPE dynamixel_series[N_JOINT] = {
  NX_DXL, NX_DXL, NX_DXL, NX_DXL, NX_DXL, NX_DXL,
  NX_DXL, NX_DXL, NX_DXL, NX_DXL, NX_DXL, NX_DXL,
  MX_DXL, MX_DXL, MX_DXL, 
  MX_DXL, MX_DXL, MX_DXL
};

//zero position offset arrays
const int dynamixel_offset[N_JOINT] = {
  0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0,
  0, 0, 0,
  0, 0, 0
};

#endif
