/* 
  Lua interface to Statics.cpp
*/

#include "Statics.h"
#include "luaStatics.h"

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
  /* forward statics to convert servo forces to joint torques */
  std::vector<double> v = lua_checkvector(L, 1); // servo forces
  std::vector<double> q = lua_checkvector(L, 2); // joint angles
  std::vector<double> r = lua_checkvector(L, 3); // servo positions 
  std::vector<double> t = statics_forward_joints(&v[0], &q[0], &r[0]);
  lua_pushvector(L, t);
  return 1;

}

static int inverse_joints(lua_State *L)
{
  /* inverse statics to convert joint torques to servo forces */
  std::vector<double> t = lua_checkvector(L, 1); // joint torques
  std::vector<double> q = lua_checkvector(L, 2); // joint angles
  std::vector<double> r = lua_checkvector(L, 3); // servo positions 
  std::vector<double> v = statics_inverse_joints(&t[0], &q[0], &r[0]);
  lua_pushvector(L, v);
  return 1;
}

static const struct luaL_reg statics_lib [] = {
  {"forward_joints", forward_joints},
  {"inverse_joints", inverse_joints},
  {NULL, NULL}
};

extern "C"
int luaopen_Statics (lua_State *L) {
  luaL_register(L, "Statics", statics_lib);
  return 1;
}
