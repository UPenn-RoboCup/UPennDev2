/* 
  Lua interface to Kinematics.cpp
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

static int forward_joints(lua_State *L)
{
  /* forward kinematics to convert servo positions to joint angles */
  std::vector<double> r = lua_checkvector(L, 1);
  std::vector<double> q = kinematics_forward_joints(&r[0]);
  lua_pushvector(L, q);
  return 1;

}

static int inverse_joints(lua_State *L)
{
  /* inverse kinematics to convert joint angles to servo positions */
  std::vector<double> q = lua_checkvector(L, 1);
  std::vector<double> r = kinematics_inverse_joints(&q[0]);
  lua_pushvector(L, r);
  return 1;
}

static const struct luaL_reg kinematics_lib [] = {
  {"forward_joints", forward_joints},
  {"inverse_joints", inverse_joints},
  {NULL, NULL}
};

extern "C"
int luaopen_Kinematics (lua_State *L) {
  luaL_register(L, "Kinematics", kinematics_lib);
  
  return 1;
}
