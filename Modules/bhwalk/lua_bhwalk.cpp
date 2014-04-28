/*
   Lua BHuman Wrapper
   (c) 2014 Stephen McGill
   */
#include <lua.hpp>
#include "WalkingEngine.h"

// Out state variables in C
// TODO: These should kinda be in Lua. Let's do it later, though
// NOTE: The general usage of WalkgingEngine is how Bowdoin does it
// NOTE: From BHWalkProvider
static WalkingEngine walkingEngine;
static bool requestedToStop = false;

const float INITIAL_BODY_POSE_ANGLES[] =
{
  1.57f, 0.18f, -1.56f, -0.18f,
  0.0f, 0.0f, -0.39f, 0.76f, -0.37f, 0.0f,
  0.0f, 0.0f, -0.39f, 0.76f, -0.37f, 0.0f,
  1.57f, -0.18f, 1.43f, 0.23f
};

bool hasLargerMagnitude(float x, float y) {
  if (y > 0.0f)
    return x > y;
  if (y < 0.0f)
    return x < y;
  return true; // considers values of 0.0f as always smaller in magnitude than anything
}

/* return true if p1 has "passed" p2 (has components values that either have a magnitude
   larger than the corresponding magnitude p2 within the same sign)
   */
bool hasPassed(const Pose2D& p1, const Pose2D& p2) {

  return (hasLargerMagnitude(p1.rotation, p2.rotation) &&
          hasLargerMagnitude(p1.translation.x, p2.translation.x) &&
          hasLargerMagnitude(p1.translation.y, p2.translation.y));
}

static int luaBH_is_standing (lua_State *L) {
  lua_pushboolean(
    L,
    walkingEngine.theMotionRequest.motion == MotionRequest::stand
  );
  return 1;
}

static int luaBH_is_walk_active (lua_State *L) {
  lua_pushboolean(
    L,
    !(isStanding() && walkingEngine.walkingEngineOutput.isLeavingPossible) && isActive()
  );
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

static int luaBH_hard_reset (lua_State *L) {
  inactive();

  // reset odometry
  walkingEngine.theOdometryData = OdometryData();

  // Set the motion request
  MotionRequest motionRequest;
  motionRequest.motion = MotionRequest::specialAction;

  motionRequest.specialActionRequest.specialAction = SpecialActionRequest::standUpBackNao;
  currentCommand = MotionCommand::ptr();

  walkingEngine.inertiaSensorCalibrator.reset();

  requestedToStop = false;
  // Nothing pushed
  return 0;
}

// Request to stand
static int luaBH_stand (lua_State *L) {
    currentCommand = MotionCommand::ptr();
    active();
    return 0;
}

static const struct luaL_Reg bhwalk_lib [] = {
  {"is_standing", luaBH_is_standing},
  {"is_walk_active", luaBH_is_walk_active},
  {"is_calibrated", luaBH_is_calibrated},
  {"get_hand_speeds", luaBH_get_hand_speeds},
  {"hard_reset", luaBH_hard_reset},
  {"stand", luaBH_stand},
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
