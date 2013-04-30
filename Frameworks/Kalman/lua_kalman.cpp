#include <lua.hpp>
#include <math.h>
#include "BallModel.h"

const double MIN_ERROR_DISTANCE = 50;
const double ERROR_DEPTH_FACTOR = 0.08;
const double ERROR_ANGLE = 3*M_PI/180;

static int lua_get_ball(lua_State *L) {

  static BallModel bm; // Keep the model of the ball static

  double xObject = lua_tonumber(L, 1);
  double yObject = lua_tonumber(L, 2);
  double uncertainty = lua_tonumber(L, 3);

  double distance = sqrt(xObject*xObject+yObject*yObject);
  double angle = atan2(-xObject, yObject);

  double errorDepth = ERROR_DEPTH_FACTOR*(distance+MIN_ERROR_DISTANCE);
  double errorAzimuthal = ERROR_ANGLE*(distance+MIN_ERROR_DISTANCE);

  Gaussian2d objGaussian;
  objGaussian.setMean(xObject, yObject);
  objGaussian.setCovarianceAxis(errorAzimuthal, errorDepth, angle);

  bm.BallObservation(objGaussian, (int)(uncertainty));

  double x,y,vx,vy,ex,evx;
  bm.getBall( x, y, vx, vy, ex, evx);
  lua_pushnumber(L, x);
  lua_pushnumber(L, y);
  lua_pushnumber(L, vx);
  lua_pushnumber(L, vy);
  lua_pushnumber(L, ex);
  lua_pushnumber(L, evx);

  return 6;
}

static const struct luaL_Reg kalman_lib [] = {
  {"get_ball", lua_get_ball},
  {NULL, NULL}
};

extern "C"
int luaopen_kalman (lua_State *L) {
  
#if LUA_VERSION_NUM == 502
  luaL_newlib(L, kalman_lib);
#else
	luaL_register(L, "kalman", kalman_lib);
#endif
  return 1;
}

