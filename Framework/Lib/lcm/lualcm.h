#ifndef _LUA_LCM_H_
#define _LUA_LCM_H_

#include "lua.h"
#include "lualib.h"
#include "lauxlib.h"
#include <lcm/lcm.h>

#define MAX_HANDLERS 128

#define lua_checklcm(L, narg) \
  (lua_lcm_t *)luaL_checkudata(L, narg, "lcm_mt")

typedef struct _lua_lcm_handler_t {
  lua_State *L;
  int reference;
} lua_lcm_handler_t;

typedef struct _lua_lcm_t {
  lcm_t *lcm;
  lua_lcm_handler_t handlers[MAX_HANDLERS];
  int n_handlers;
} lua_lcm_t;

int luaopen_lcm(lua_State *L);

#endif
