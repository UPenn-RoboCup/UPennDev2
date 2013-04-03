#ifndef luaHokuyo_h_DEFINED
#define luaHokuyo_H_DEFINED

#include "VisDataTypes.hh"

using namespace vis;

extern "C"
{
#include "lua.h"
#include "lualib.h"
#include "lauxlib.h"
}

extern "C"
int luaopen_Hokuyo(lua_State *L);

struct LidarScan
{
  uint32_t      counter;
  uint32_t      id;
  FloatArray    ranges ;
//  UInt16Array   intensities ;

  float         startAngle ; // radians
  float         stopAngle ;  // radians
  float         angleStep ;  // radians
  double        startTime ;  // seconds
  double        stopTime ;   // seconds

};

#endif
