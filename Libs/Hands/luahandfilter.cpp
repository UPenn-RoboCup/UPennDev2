#include "luahandfilter.h"

const double MIN_ERROR_DISTANCE = 50;
const double ERROR_DEPTH_FACTOR = 0.08;
const double ERROR_ANGLE = 3*PI/180;

static BallModel right_hand;
static BallModel left_hand;

void update_model( 
    int bm, double x,double y, int uncertainty ){

  double distance = sqrt(x*x+y*y);
  double angle = atan2(-x, y);

  double errorDepth = ERROR_DEPTH_FACTOR*(distance+MIN_ERROR_DISTANCE);
  double errorAzimuthal = ERROR_ANGLE*(distance+MIN_ERROR_DISTANCE);

  Gaussian2d objGaussian;
  objGaussian.setMean(x, y);
  objGaussian.setCovarianceAxis(errorAzimuthal, errorDepth, angle);

  if( bm==1 )
    right_hand.BallObservation(objGaussian, uncertainty);
  else
    left_hand.BallObservation(objGaussian, uncertainty);

}

static int lua_get_right_hand(lua_State *L) {
  double x_obs = lua_tonumber(L, 1);
  double y_obs = lua_tonumber(L, 2);
  int uncertainty = (int)lua_tonumber(L, 3);

  update_model(1,x_obs,y_obs,uncertainty);

  double x,y,vx,vy,ex,evx;
  right_hand.getBall( x, y, vx, vy, ex, evx);
  lua_pushnumber(L, x);
  lua_pushnumber(L, y);
  lua_pushnumber(L, vx);
  lua_pushnumber(L, vy);
  lua_pushnumber(L, ex);
  lua_pushnumber(L, evx);

  return 6;
}
static int lua_get_left_hand(lua_State *L) {

  double x_obs = lua_tonumber(L, 1);
  double y_obs = lua_tonumber(L, 2);
  double uncertainty = (int)lua_tonumber(L, 3);

  update_model(0,x_obs,y_obs,uncertainty);

  double x,y,vx,vy,ex,evx;
  left_hand.getBall( x, y, vx, vy, ex, evx);
  lua_pushnumber(L, x);
  lua_pushnumber(L, y);
  lua_pushnumber(L, vx);
  lua_pushnumber(L, vy);
  lua_pushnumber(L, ex);
  lua_pushnumber(L, evx);

  return 6;
}

static const struct luaL_reg handfilter_lib [] = {
  {"get_left_hand", lua_get_left_hand},
  {"get_right_hand", lua_get_right_hand},
  {NULL, NULL}
};

extern "C"
int luaopen_handfilter (lua_State *L) {
  luaL_register(L, "handfilter", handfilter_lib);
  return 1;
}

