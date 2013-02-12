/* 
  Lua interface to THOROP Kinematics

  To compile on Mac OS X:
  g++ -arch i386 -o THOROPKinematics.dylib -bundle -undefined dynamic_lookup luaTHOROPKinematics.pp THOROPKinematics.cc Transform.cc -lm
*/

#include "THOROPKinematics.h"
#include "luaTHOROPKinematics.h"

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

static int forward_joints(lua_State *L)
{
  /* forward kinematics to convert servo positions to joint angles */
  std::vector<double> r = lua_checkvector(L, 1);
  std::vector<double> q = THOROP_kinematics_forward_joints(&r[0]);
  lua_pushvector(L, q);
  return 1;

}

static int forward_head(lua_State *L) {
  std::vector<double> q = lua_checkvector(L, 1);
  Transform t = THOROP_kinematics_forward_head(&q[0]);
  lua_pushtransform(L, t);
  return 1;
}

static int forward_l_arm(lua_State *L) {
  std::vector<double> q = lua_checkvector(L, 1);
  Transform t = THOROP_kinematics_forward_l_arm(&q[0]);
  lua_pushtransform(L, t);
  return 1;
}

static int forward_r_arm(lua_State *L) {
  std::vector<double> q = lua_checkvector(L, 1);
  Transform t = THOROP_kinematics_forward_r_arm(&q[0]);
  lua_pushtransform(L, t);
  return 1;
}

static int forward_l_leg(lua_State *L) {
  std::vector<double> q = lua_checkvector(L, 1);
  Transform t = THOROP_kinematics_forward_l_leg(&q[0]);
  lua_pushtransform(L, t);
  return 1;
}

static int forward_r_leg(lua_State *L) {
  std::vector<double> q = lua_checkvector(L, 1);
  Transform t = THOROP_kinematics_forward_r_leg(&q[0]);
  lua_pushtransform(L, t);
  return 1;
}

static int l_arm_torso(lua_State *L) {
  std::vector<double> q = lua_checkvector(L, 1);
  Transform t = THOROP_kinematics_forward_l_arm(&q[0]);
  lua_pushvector(L, position6D(t));
  return 1;
}

static int torso_l_arm(lua_State *L) {
  std::vector<double> q = lua_checkvector(L, 1);
  Transform t = inv(THOROP_kinematics_forward_l_arm(&q[0]));
  lua_pushvector(L, position6D(t));
  return 1;
}

static int r_arm_torso(lua_State *L) {
  std::vector<double> q = lua_checkvector(L, 1);
  Transform t = THOROP_kinematics_forward_r_arm(&q[0]);
  lua_pushvector(L, position6D(t));
  return 1;
}

static int torso_r_arm(lua_State *L) {
  std::vector<double> q = lua_checkvector(L, 1);
  Transform t = inv(THOROP_kinematics_forward_r_arm(&q[0]));
  lua_pushvector(L, position6D(t));
  return 1;
}

static int l_leg_torso(lua_State *L) {
  std::vector<double> q = lua_checkvector(L, 1);
  Transform t = THOROP_kinematics_forward_l_leg(&q[0]);
  lua_pushvector(L, position6D(t));
  return 1;
}

static int torso_l_leg(lua_State *L) {
  std::vector<double> q = lua_checkvector(L, 1);
  Transform t = inv(THOROP_kinematics_forward_l_leg(&q[0]));
  lua_pushvector(L, position6D(t));
  return 1;
}

static int r_leg_torso(lua_State *L) {
  std::vector<double> q = lua_checkvector(L, 1);
  Transform t = THOROP_kinematics_forward_r_leg(&q[0]);
  lua_pushvector(L, position6D(t));
  return 1;
}

static int torso_r_leg(lua_State *L) {
  std::vector<double> q = lua_checkvector(L, 1);
  Transform t = inv(THOROP_kinematics_forward_r_leg(&q[0]));
  lua_pushvector(L, position6D(t));
  return 1;
}

static int inverse_joints(lua_State *L)
{
  /* inverse kinematics to convert joint angles to servo positions */
  std::vector<double> q = lua_checkvector(L, 1);
  std::vector<double> r = THOROP_kinematics_inverse_joints(&q[0]);
  lua_pushvector(L, r);
  return 1;
}

static int inverse_l_arm(lua_State *L) {
  std::vector<double> qArm;
  std::vector<double> pArm = lua_checkvector(L, 1);
  Transform trArm = transform6D(&pArm[0]);
  qArm = THOROP_kinematics_inverse_l_arm(trArm);
  lua_pushvector(L, qArm);
  return 1;
}

static int inverse_r_arm(lua_State *L) {
  std::vector<double> qArm;
  std::vector<double> pArm = lua_checkvector(L, 1);
  Transform trArm = transform6D(&pArm[0]);
  qArm = THOROP_kinematics_inverse_r_arm(trArm);
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

  qLArm = THOROP_kinematics_inverse_l_arm(trTorso_LArm);
  qRArm = THOROP_kinematics_inverse_r_arm(trTorso_RArm);
  qLArm.insert(qLArm.end(), qRArm.begin(), qRArm.end());

  lua_pushvector(L, qLArm);
  return 1;
}

static int inverse_l_leg(lua_State *L) {
  std::vector<double> qLeg;
  std::vector<double> pLeg = lua_checkvector(L, 1);
  Transform trLeg = transform6D(&pLeg[0]);
  qLeg = THOROP_kinematics_inverse_r_leg(trLeg);
  lua_pushvector(L, qLeg);
  return 1;
}

static int inverse_r_leg(lua_State *L) {
  std::vector<double> qLeg;
  std::vector<double> pLeg = lua_checkvector(L, 1);
  Transform trLeg = transform6D(&pLeg[0]);
  qLeg = THOROP_kinematics_inverse_l_leg(trLeg);
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

  qLLeg = THOROP_kinematics_inverse_l_leg(trTorso_LLeg);
  qRLeg = THOROP_kinematics_inverse_r_leg(trTorso_RLeg);
  qLLeg.insert(qLLeg.end(), qRLeg.begin(), qRLeg.end());

  lua_pushvector(L, qLLeg);
  return 1;
}

static const struct luaL_reg kinematics_lib [] = {
  {"forward_head", forward_head},
  {"forward_larm", forward_l_arm},
  {"forward_rarm", forward_r_arm},
  {"forward_lleg", forward_l_leg},
  {"forward_rleg", forward_r_leg},
  {"forward_joints", forward_joints},
  {"lleg_torso", l_leg_torso},
  {"torso_lleg", torso_l_leg},
  {"rleg_torso", r_leg_torso},
  {"torso_rleg", torso_r_leg},
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
  {"inverse_joints", inverse_joints},
  {NULL, NULL}
};

extern "C"
int luaopen_THOROPKinematics (lua_State *L) {
  luaL_register(L, "THOROPKinematics", kinematics_lib);
  
  return 1;
}
