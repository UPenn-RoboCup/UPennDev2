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
	std::vector<double> pArm = lua_checkvector(L, 1);
	//Transform trArm = transform6D(&pArm[0]);
	qArm = YouBot_kinematics_inverse_arm(&pArm[0]);
	lua_pushvector(L, qArm);
	return 1;
}

static const struct luaL_Reg kinematics_lib [] = {
	{"forward_arm", forward_arm},
	{"inverse_arm", inverse_arm},
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

