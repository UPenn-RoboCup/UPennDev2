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

static int lua_is_standing( lua_State *L ) {
  lua_pushboolean(
                  L,
                  walkingEngine.theMotionRequest.motion == MotionRequest::stand
                 );
  return 1;
}

static const struct luaL_Reg bhwalk_lib [] = {
  {"is_standing", lua_is_standing},
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
