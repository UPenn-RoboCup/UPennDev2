/* 
(c) 2013 Seung Joon Yi
7 DOF
*/

#include <lua.hpp>
#include "YouBotKinematics.h"

/* Copied from lua_unix */
struct def_info {
  const char *name;
  double value;
};

void lua_install_constants(lua_State *L, const struct def_info constants[]) {
  int i;
  for (i = 0; constants[i].name; i++) {
    lua_pushstring(L, constants[i].name);
    lua_pushnumber(L, constants[i].value);
    lua_rawset(L, -3);
  }
}

static void lua_pushvector(lua_State *L, std::vector<double> v) {
	int n = v.size();
	lua_createtable(L, n, 0);
	for (int i = 0; i < n; i++) {
		lua_pushnumber(L, v[i]);
		lua_rawseti(L, -2, i+1);
	}
}

static std::vector<double> lua_checkvector(lua_State *L, int narg) {
	/*
	if (!lua_istable(L, narg))
	luaL_typerror(L, narg, "vector");
	*/
	if ( !lua_istable(L, narg) )
		luaL_argerror(L, narg, "vector");

#if LUA_VERSION_NUM == 502
	int n = lua_rawlen(L, narg);
#else	
	int n = lua_objlen(L, narg);
#endif
	std::vector<double> v(n);
	for (int i = 0; i < n; i++) {
		lua_rawgeti(L, narg, i+1);
		v[i] = lua_tonumber(L, -1);
		lua_pop(L, 1);
	}
	return v;
}

#ifdef TORCH
static std::vector<double> lua_checktransform(lua_State *L, int narg) {
  std::vector<double> v(16);
  return v;
}
#endif

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


static int forward_arm(lua_State *L) {
	std::vector<double> q = lua_checkvector(L, 1);
	Transform t = YouBot_kinematics_forward_arm(&q[0]);
	lua_pushtransform(L, t);
	return 1;
}

static int inverse_arm(lua_State *L) {
	std::vector<double> qArm;

  #ifdef TORCH
  const THDoubleTensor * tr =
		(THDoubleTensor *) luaT_checkudata(L, 1, "torch.DoubleTensor");
  #else
  std::vector<double> pArm = lua_checkvector(L, 1);
  const double * tr = &pArm[0];
  //Transform tr = transform6D(&pArm[0]);
  #endif

  qArm = YouBot_kinematics_inverse_arm( tr );
	lua_pushvector(L, qArm);
	return 1;
}

static int test(lua_State *L) {
  /*
  // This seems good...
  THLongStorage *storage = THLongStorage_newWithSize(2);
	luaT_pushudata(L, storage, "torch.LongStorage");
  */

  // This also seems to work!
  THLongStorage *sz = THLongStorage_newWithSize(2);
  sz->data[0] = 4;
  sz->data[1] = 4;
  THDoubleTensor *storage = THDoubleTensor_newWithSize(sz,NULL);
	luaT_pushudata(L, storage, "torch.DoubleTensor");

	return 1;
}

static const struct luaL_Reg kinematics_lib [] = {
	{"forward_arm", forward_arm},
	{"inverse_arm", inverse_arm},
  {"test", test},
	{NULL, NULL}
};

static const def_info kinematics_constants[] = {
  {"upperArmLength", upperArmLength},
  {"lowerArmLength", lowerArmLength},
  {NULL, 0}
};

extern "C"
int luaopen_YouBotKinematics (lua_State *L) {
#if LUA_VERSION_NUM == 502
	luaL_newlib(L, kinematics_lib);
#else
	luaL_register(L, "Kinematics", kinematics_lib);
#endif
	lua_install_constants(L, kinematics_constants);
	return 1;
}

