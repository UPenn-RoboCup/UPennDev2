/*
   Lua BHuman Wrapper
   (c) 2014 Stephen McGill
   */
#include <lua.hpp>
#include "WalkingEngine.h"

static WalkingEngine walkingEngine;

const float INITIAL_BODY_POSE_ANGLES[] =
{
  1.57f, 0.18f, -1.56f, -0.18f,
  0.0f, 0.0f, -0.39f, 0.76f, -0.37f, 0.0f,
  0.0f, 0.0f, -0.39f, 0.76f, -0.37f, 0.0f,
  1.57f, -0.18f, 1.43f, 0.23f
};

static int luaBH_get_motion_request (lua_State *L) {
  lua_pushnumber(L, walkingEngine.theMotionRequest.motion);
  return 1;
}

static int luaBH_is_leaving_possible (lua_State *L) {
  lua_pushboolean(L, walkingEngine.walkingEngineOutput.isLeavingPossible);
  return 1;
}

static int luaBH_is_calibrated (lua_State *L) {
  lua_pushboolean(L, walkingEngine.theInertiaSensorData.calibrated);
  return 1;
}

static int luaBH_get_hand_speeds (lua_State *L) {
  lua_pushnumber(L, walkingEngine.leftHandSpeed);
  lua_pushnumber(L, walkingEngine.rightHandSpeed);
  return 2;
}

static int luaBH_get_odometry (lua_State *L) {
  lua_pushnumber(L, walkingEngine.theOdometryData.translation.x);
  lua_pushnumber(L, walkingEngine.theOdometryData.translation.y);
  lua_pushnumber(L, walkingEngine.theOdometryData.rotation);
  return 3;
}

static const struct luaL_Reg bhwalk_lib [] = {
  {"get_motion_request", luaBH_get_motion_request},
  {"is_leaving_possible", luaBH_is_leaving_possible},
  {"is_calibrated", luaBH_is_calibrated},
  {"get_hand_speeds", luaBH_get_hand_speeds},
  {"get_odometry", luaBH_get_odometry},
  {NULL, NULL}
};

extern "C" int luaopen_bhwalk(lua_State *L) {
#if LUA_VERSION_NUM == 502
  luaL_newlib(L, bhwalk_lib);
#else
  luaL_register(L, "bhwalk", bhwalk_lib);
#endif
  return 1;
}
