#include <math.h>
#include <stdint.h>
#include "Microstrain.h"
#include "luamicrostrain.h"

Microstrain imu;

static int lua_microstrain_open(lua_State *L)
{
  const char* port_name = luaL_checkstring(L, 1);
  imu.openPort(port_name);
  imu.initTime(0);
  return 0;
}

static int lua_microstrain_init_gyros(lua_State *L)
{
  double bias_x;
  double bias_y;
  double bias_z;
  imu.initGyros(&bias_x, &bias_y, &bias_z);
  lua_pushnumber(L, bias_x);
  lua_pushnumber(L, bias_y);
  lua_pushnumber(L, bias_z);
  return 3;
}

static int lua_microstrain_set_continuous(lua_State *L)
{
  int enable = luaL_checkint(L, 1);
  if (enable) {
    bool ret = imu.setContinuous(Microstrain::CMD_ACCEL_ANGRATE_ORIENT);
    lua_pushboolean(L, ret);
  }
  else {
    imu.stopContinuous();
    lua_pushboolean(L, true);
  }
  return 1;
}

static int lua_microstrain_request_data(lua_State *L)
{
  uint8_t cmd[] = {Microstrain::CMD_ACCEL_ANGRATE_ORIENT};
  imu.send(cmd, sizeof(cmd));
  return 0;
}

static int lua_microstrain_receive_data(lua_State *L)
{
  uint64_t time;
  double accel[3], gyro[3], orientation[9];
  imu.receiveAccelAngrateOrientation(&time, accel, gyro, orientation);
  lua_createtable(L, 9, 0);
  lua_pushnumber(L, accel[0]);                              // x accel
  lua_rawseti(L, -2, 1);
  lua_pushnumber(L, accel[1]);                              // y accel
  lua_rawseti(L, -2, 2);
  lua_pushnumber(L, -accel[2]);                             // z accel
  lua_rawseti(L, -2, 3);
  lua_pushnumber(L, gyro[0]);                               // pitch gyro
  lua_rawseti(L, -2, 4);
  lua_pushnumber(L, gyro[1]);                               // roll gyro
  lua_rawseti(L, -2, 5);
  lua_pushnumber(L, gyro[2]);                              // yaw gyro
  lua_rawseti(L, -2, 6);
  lua_pushnumber(L, atan(orientation[5]/orientation[8]));   // pitch
  lua_rawseti(L, -2, 7);
  lua_pushnumber(L, -asin(orientation[2]));                 // roll
  lua_rawseti(L, -2, 8);
  lua_pushnumber(L, -atan2(orientation[1],orientation[0])); // yaw
  lua_rawseti(L, -2, 9);
  return 1;
}

static int lua_microstrain_close(lua_State *L)
{
  imu.closePort();
  return 0;
}

static const struct luaL_reg microstrain_functions [] =
{
  {"open", lua_microstrain_open},
  {"init_gyros", lua_microstrain_init_gyros},
  {"set_continuous", lua_microstrain_set_continuous},
  {"request_data", lua_microstrain_request_data},
  {"receive_data", lua_microstrain_receive_data},
  {"close", lua_microstrain_close},
  {NULL, NULL},
};

extern "C"
int luaopen_microstrain(lua_State *L)
{
  luaL_register(L, "microstrain", microstrain_functions);
  return 1;
}
