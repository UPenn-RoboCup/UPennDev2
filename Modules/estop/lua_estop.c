#include "lua.h"
#include "lualib.h"
#include "lauxlib.h"

#include <wchar.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdio.h>

#include "SerialInterface.h"
#include "VehicleInterface.h"
#include "estop.h"

#define MT_NAME "estop_mt"


static void lua_pushintarray(lua_State *L, int* v, int n) {
  lua_createtable(L, n, 0);
  int i=0;
  for (i = 0; i < n; i++) {
    lua_pushnumber(L, v[i]);
    lua_rawseti(L, -2, i+1);
  }
}

static int lua_estop_index(lua_State *L) {
  /* Get index through metatable: */
  if (!lua_getmetatable(L, 1)) {lua_pop(L, 1); return 0;} /* push metatable */
  lua_pushvalue(L, 2); /* copy key */
  lua_rawget(L, -2); /* get metatable function */
  lua_remove(L, -2); /* delete metatable */
  return 1;
}

static int lua_estop_delete(lua_State *L) { 
  estop_shutdown();
  return 1;
}

static int lua_estop_init(lua_State *L) {
  estop_init("/dev/ttyACM1",115200);
//  estop_init("/dev/ttyACM0",115200);
  return 1;
}


static int lua_estop_update(lua_State *L) {
  int lstick[4],rstick[4],lbutton[4],rbutton[4];
  int ret = estop_update(lstick,rstick,lbutton,rbutton);  
  int rb = rbutton[1]+ 4*rbutton[2] + 3*rbutton[0] + 2*rbutton[3];

  lua_createtable(L, 0, 4);
  lua_pushstring(L, "estop");
    lua_pushnumber(L,ret);
  lua_rawset(L,-3);

  lua_pushstring(L, "lbutton");
    lua_pushintarray(L,lbutton,4);
  lua_rawset(L,-3);

  lua_pushstring(L, "lstick");
    lua_pushintarray(L,lstick,3);
  lua_rawset(L,-3);

  lua_pushstring(L, "rstick");
    lua_pushintarray(L,rstick,3);
  lua_rawset(L,-3);

  lua_pushstring(L, "rbutton");
    lua_pushnumber(L,rb);
  lua_rawset(L,-3);  
  return 1;
}

static int lua_estop_display(lua_State *L){
  int row = luaL_optnumber(L, 1,1);
  const char * text= luaL_checkstring (L, 2);

  estop_display(row,text);
  return 1;
}


static const struct luaL_reg estop_functions [] = {
  {"init", lua_estop_init},
  {"update", lua_estop_update},
  {"display", lua_estop_display},
  {NULL, NULL}
};

static const struct luaL_reg estop_methods [] = {
  //{"get_raw", lua_spnav_get_raw},
  //{"get", lua_spnav_get},
  {"__gc", lua_estop_delete},  
  {"__index", lua_estop_index},
  {NULL, NULL}
};

int luaopen_estop(lua_State *L) {
  luaL_newmetatable(L, MT_NAME);

#if LUA_VERSION_NUM == 502
  luaL_setfuncs(L, estop_methods, 0);
  luaL_newlib(L, estop_functions);
#else
  luaL_register(L, NULL, estop_methods);
  luaL_register(L, "estop", estop_functions);
#endif

  return 1;
}
