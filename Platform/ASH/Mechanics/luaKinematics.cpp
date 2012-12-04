/* 
  Lua interface to SAFFiR Kinematics

  To compile on Mac OS X:
  g++ -arch i386 -o Kinematics.dylib -bundle -undefined dynamic_lookup luaKinematics.pp Kinematics.cc Transform.cc -lm
*/

#include "Kinematics.h"
#include "luaKinematics.h"

#ifdef __cplusplus
extern "C" {
#endif

#include "lua.h"
#include "lualib.h"
#include "lauxlib.h"

#ifdef __cplusplus
}
#endif


static void lua_pushvector(lua_State *L, std::vector<double> v) {
  int n = v.size();
  lua_createtable(L, n, 0);
  for (int i = 0; i < n; i++) {
    lua_pushnumber(L, v[i]);
    lua_rawseti(L, -2, i+1);
  }
}

static std::vector<double> lua_checkvector(lua_State *L, int narg) {
  if (!lua_istable(L, narg))
    luaL_typerror(L, narg, "vector");
  int n = lua_objlen(L, narg);
  std::vector<double> v(n);
  for (int i = 0; i < n; i++) {
    lua_rawgeti(L, narg, i+1);
    v[i] = lua_tonumber(L, -1);
    lua_pop(L, 1);
  }
  return v;
}

static void lua_pushtransform(lua_State *L, Transform t) {
  lua_createtable(L, 4, 0);
  for (int i = 0; i < 4; i++) {
    lua_createtable(L, 4, 0);
    for (int j = 0; j < 4; j++) {
      lua_pushnumber(L, t(i,j));
      lua_rawseti(L, -2, j+1);
    }
    lua_rawseti(L, -2, i+1);
  }
}

static int forward_head(lua_State *L) {
  std::vector<double> q = lua_checkvector(L, 1);
  Transform t = kinematics_forward_head(&q[0]);
  lua_pushtransform(L, t);
  return 1;
}

static int forward_l_arm(lua_State *L) {
  std::vector<double> q = lua_checkvector(L, 1);
  Transform t = kinematics_forward_l_arm(&q[0]);
  lua_pushtransform(L, t);
  return 1;
}

static int forward_r_arm(lua_State *L) {
  std::vector<double> q = lua_checkvector(L, 1);
  Transform t = kinematics_forward_r_arm(&q[0]);
  lua_pushtransform(L, t);
  return 1;
}

static int forward_l_leg(lua_State *L) {
  std::vector<double> q = lua_checkvector(L, 1);
  Transform t = kinematics_forward_l_leg(&q[0]);
  lua_pushtransform(L, t);
  return 1;
}

static int forward_r_leg(lua_State *L) {
  std::vector<double> q = lua_checkvector(L, 1);
  Transform t = kinematics_forward_r_leg(&q[0]);
  lua_pushtransform(L, t);
  return 1;
}

static int forward_legs(lua_State *L) {
  /* returns left and right foot tranforms with respect to the base frame */
  std::vector<double> qLegs = lua_checkvector(L, 1);
  std::vector<double> pTorso = lua_checkvector(L, 2);
  Transform trTorso = transform6D(&pTorso[0]);
  Transform trLLeg = kinematics_forward_l_leg(&qLegs[0]);
  Transform trRLeg = kinematics_forward_r_leg(&qLegs[6]);
  trLLeg = trTorso*trLLeg;
  trRLeg = trTorso*trRLeg;
  lua_pushtransform(L, trLLeg);
  lua_pushtransform(L, trRLeg);
  return 2;
}

static int l_arm_torso(lua_State *L) {
  std::vector<double> q = lua_checkvector(L, 1);
  Transform t = kinematics_forward_l_arm(&q[0]);
  lua_pushvector(L, position6D(t));
  return 1;
}

static int torso_l_arm(lua_State *L) {
  std::vector<double> q = lua_checkvector(L, 1);
  Transform t = inv(kinematics_forward_l_arm(&q[0]));
  lua_pushvector(L, position6D(t));
  return 1;
}

static int r_arm_torso(lua_State *L) {
  std::vector<double> q = lua_checkvector(L, 1);
  Transform t = kinematics_forward_r_arm(&q[0]);
  lua_pushvector(L, position6D(t));
  return 1;
}

static int torso_r_arm(lua_State *L) {
  std::vector<double> q = lua_checkvector(L, 1);
  Transform t = inv(kinematics_forward_r_arm(&q[0]));
  lua_pushvector(L, position6D(t));
  return 1;
}

static int l_leg_torso(lua_State *L) {
  std::vector<double> q = lua_checkvector(L, 1);
  Transform t = kinematics_forward_l_leg(&q[0]);
  lua_pushvector(L, position6D(t));
  return 1;
}

static int torso_l_leg(lua_State *L) {
  std::vector<double> q = lua_checkvector(L, 1);
  Transform t = inv(kinematics_forward_l_leg(&q[0]));
  lua_pushvector(L, position6D(t));
  return 1;
}

static int r_leg_torso(lua_State *L) {
  std::vector<double> q = lua_checkvector(L, 1);
  Transform t = kinematics_forward_r_leg(&q[0]);
  lua_pushvector(L, position6D(t));
  return 1;
}

static int torso_r_leg(lua_State *L) {
  std::vector<double> q = lua_checkvector(L, 1);
  Transform t = inv(kinematics_forward_r_leg(&q[0]));
  lua_pushvector(L, position6D(t));
  return 1;
}

static int inverse_l_arm(lua_State *L) {
  std::vector<double> qArm;
  std::vector<double> pArm = lua_checkvector(L, 1);
  Transform trArm = transform6D(&pArm[0]);
  qArm = kinematics_inverse_r_arm(trArm);
  lua_pushvector(L, qArm);
  return 1;
}

static int inverse_r_arm(lua_State *L) {
  std::vector<double> qArm;
  std::vector<double> pArm = lua_checkvector(L, 1);
  Transform trArm = transform6D(&pArm[0]);
  qArm = kinematics_inverse_l_arm(trArm);
  lua_pushvector(L, qArm);
  return 1;
}

static int inverse_arms(lua_State *L) {
  std::vector<double> qLArm(12), qRArm;
  std::vector<double> pLArm = lua_checkvector(L, 1);
  std::vector<double> pRArm = lua_checkvector(L, 2);
  std::vector<double> pTorso = lua_checkvector(L, 3);

  Transform trLArm = transform6D(&pLArm[0]);
  Transform trRArm = transform6D(&pRArm[0]);
  Transform trTorso = transform6D(&pTorso[0]);
  Transform trTorso_LArm = inv(trTorso)*trLArm;
  Transform trTorso_RArm = inv(trTorso)*trRArm;

  qLArm = kinematics_inverse_l_arm(trTorso_LArm);
  qRArm = kinematics_inverse_r_arm(trTorso_RArm);
  qLArm.insert(qLArm.end(), qRArm.begin(), qRArm.end());

  lua_pushvector(L, qLArm);
  return 1;
}

static int inverse_l_leg(lua_State *L) {
  std::vector<double> qLeg;
  std::vector<double> pLeg = lua_checkvector(L, 1);
  Transform trLeg = transform6D(&pLeg[0]);
  qLeg = kinematics_inverse_l_leg(trLeg);
  lua_pushvector(L, qLeg);
  return 1;
}

static int inverse_r_leg(lua_State *L) {
  std::vector<double> qLeg;
  std::vector<double> pLeg = lua_checkvector(L, 1);
  Transform trLeg = transform6D(&pLeg[0]);
  qLeg = kinematics_inverse_r_leg(trLeg);
  lua_pushvector(L, qLeg);
  return 1;
}

static int inverse_legs(lua_State *L) {
  std::vector<double> qLLeg(12), qRLeg;
  std::vector<double> pLLeg = lua_checkvector(L, 1);
  std::vector<double> pRLeg = lua_checkvector(L, 2);
  std::vector<double> pTorso = lua_checkvector(L, 3);

  Transform trLLeg = transform6D(&pLLeg[0]);
  Transform trRLeg = transform6D(&pRLeg[0]);
  Transform trTorso = transform6D(&pTorso[0]);
  Transform trTorso_LLeg = inv(trTorso)*trLLeg;
  Transform trTorso_RLeg = inv(trTorso)*trRLeg;

  qLLeg = kinematics_inverse_l_leg(trTorso_LLeg);
  qRLeg = kinematics_inverse_r_leg(trTorso_RLeg);
  qLLeg.insert(qLLeg.end(), qRLeg.begin(), qRLeg.end());

  lua_pushvector(L, qLLeg);
  return 1;
}

static const struct luaL_reg kinematics_lib [] = {
  {"forward_head", forward_head},
  {"forward_l_arm", forward_l_arm},
  {"forward_r_arm", forward_r_arm},
  {"forward_l_leg", forward_l_leg},
  {"forward_r_leg", forward_r_leg},
  {"forward_legs", forward_legs},
  {"l_leg_torso", l_leg_torso},
  {"torso_l_leg", torso_l_leg},
  {"r_leg_torso", r_leg_torso},
  {"torso_r_leg", torso_r_leg},
  {"l_arm_torso", l_arm_torso},
  {"torso_l_arm", torso_l_arm},
  {"r_arm_torso", r_arm_torso},
  {"torso_r_arm", torso_r_arm},
  {"inverse_l_leg", inverse_l_leg},
  {"inverse_r_leg", inverse_r_leg},
  {"inverse_legs", inverse_legs},
  {"inverse_l_arm", inverse_l_arm},
  {"inverse_r_arm", inverse_r_arm},
  {"inverse_arms", inverse_arms},
  {NULL, NULL}
};

extern "C"
int luaopen_Kinematics (lua_State *L) {
  luaL_register(L, "Kinematics", kinematics_lib);
  return 1;
}
