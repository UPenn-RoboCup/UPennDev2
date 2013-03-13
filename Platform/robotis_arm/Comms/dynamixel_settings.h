#ifndef DYNAMIXEL_SETTINGS_H 
#define DYNAMIXEL_SETTINGS_H

#include "config.h"

//motor types
enum DXL_TYPE {
  MX_DXL,
  NX_DXL
};

//mapping of dynamixel motor ids to DCM values
const int dynamixel_id[N_JOINT] = {
  1, 2, 3, 4, 5, 6,
  7, 8, 9
};

//types corresponding to each DCM mapping
const DXL_TYPE dynamixel_series[N_JOINT] = {
  NX_DXL, NX_DXL, NX_DXL, NX_DXL, NX_DXL, NX_DXL,
  MX_DXL, MX_DXL, MX_DXL
};

#endif
