/* 
(c) 2014 Stephen G. McGill
Kinematics for KUKA YouBot's 5 DOF arm
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
static Transform luaT_checktransform(lua_State *L, int narg) {
  const THDoubleTensor * _t =
		(THDoubleTensor *) luaT_checkudata(L, narg, "torch.DoubleTensor");
  // Check the dimensions
  if(_t->size[0]!=4||_t->size[1]!=4)
    luaL_error(L, "Bad dimensions: %ld x %ld",_t->size[0],_t->size[1]);

  // Form into our Transform type
  Transform tr;
  for (int i = 0; i < 4; i++)
    for (int j = 0; j < 4; j++)
      tr(i,j) = THTensor_fastGet2d( _t, i, j );

  return tr;
}
static void luaT_pushtransform(lua_State *L, Transform t) {
  // Make the Tensor
  THLongStorage *sz = THLongStorage_newWithSize(2);
  sz->data[0] = 4;
  sz->data[1] = 4;
  THDoubleTensor *_t = THDoubleTensor_newWithSize(sz,NULL);

  // Copy the data
  //double* dest = _t->storage->data;
  for (int i = 0; i < 4; i++)
    for (int j = 0; j < 4; j++)
      THTensor_fastSet2d( _t, i, j, t(i,j) );

  // Push the Tensor
	luaT_pushudata(L, _t, "torch.DoubleTensor");
}
#endif

static Transform lua_checktransform(lua_State *L, int narg) {
  // Table of tables
  luaL_checktype(L, narg, LUA_TTABLE);
#if LUA_VERSION_NUM == 502
	int n_el = lua_rawlen(L, 1);
#else
  int n_el = lua_objlen(L, 1);
#endif
  if(n_el!=4)
    luaL_error(L, "Bad dimension! %d x ?",n_el);

  // Make the Transform
  Transform tr;
  int i, j;

  // Loop through the transform
  for (i = 1; i <= 4; i++) {
    // Grab the table entry
    lua_rawgeti(L, narg, i);
    // Get the top of the stack
    int top_tbl = lua_gettop(L);
    //printf("Top of stack: %d\n",top_arg);

    luaL_checktype(L, top_tbl, LUA_TTABLE);
    #if LUA_VERSION_NUM == 502
      int n_el2 = lua_rawlen(L, 1);
    #else
      int n_el2 = lua_objlen(L, 1);
    #endif
    if(n_el!=4)
      luaL_error(L, "Bad dimension! %d x %d",i,n_el2);

    // Work with the table, which is pushed
    for (j = 1; j <= 4; j++) {
      // Grab the table entry on top of the stack (of 2 things?)
      lua_rawgeti(L, top_tbl, j);
      int top_num = lua_gettop(L);
      // The number is now on the top of the stack
      double el = luaL_checknumber(L, top_num);
      // Work with the table, which is pushed
      //printf("El @ (%d,%d)=%lf\n",i,j,el);
      tr(i-1,j-1) = el;
      // Remove the number from the stack
      lua_pop(L, 1);
    }
    // Remove from the stack
    lua_pop(L, 1);
  }

  // Return the Transform
  return tr;
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

  #ifdef TORCH
  // Check if you want a table
  if(!lua_toboolean(L, 2))
    luaT_pushtransform(L, t);
  else
  #endif
    lua_pushtransform(L, t);

	return 1;
}

// TODO: Take a Transform metatable in...?
static int inverse_arm(lua_State *L) {
	std::vector<double> qArm;

#ifdef TORCH
  if( !lua_istable(L,1) )
    qArm = YouBot_kinematics_inverse_arm( luaT_checktransform(L, 1) );
  else
#endif
    qArm = YouBot_kinematics_inverse_arm( lua_checktransform(L, 1) );
	lua_pushvector(L, qArm);
	return 1;
}

static const struct luaL_Reg kinematics_lib [] = {
	{"forward_arm", forward_arm},
	{"inverse_arm", inverse_arm},
	{NULL, NULL}
};

static const def_info kinematics_constants[] = {
  {"baseLength", baseLength},
  {"lowerArmLength", lowerArmLength},
  {"upperArmLength", upperArmLength},
  {"wristLength", wristLength},
  {"handLength", handLength},
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
