#include "lualcm.h"
#include <string.h>

static int lua_lcm_new(lua_State *L)
{
  /* create lcm object */
  const char *provider = NULL;
  lua_lcm_t *lcm = (lua_lcm_t *)lua_newuserdata(L, sizeof(lua_lcm_t));

  if (lua_type(L, 1) == LUA_TSTRING)
    provider = lua_tostring(L, 1);
  lcm->lcm = lcm_create(provider);
  if (!lcm->lcm)
    return 0;
  lcm->n_handlers = 0;

  luaL_getmetatable(L, "lcm_mt");
  lua_setmetatable(L, -2);
  return 1;
};

static int lua_lcm_get_fileno(lua_State *L)
{
  /* get file descriptor */
  lua_lcm_t *lcm = lua_checklcm(L, 1);
  int fileno = lcm_get_fileno(lcm->lcm);
  lua_pushinteger(L, fileno);
  return 1;
};

static int lua_lcm_handle(lua_State *L)
{
  /* handle incoming messages */
  lua_lcm_t *lcm = lua_checklcm(L, 1);
  int result = lcm_handle(lcm->lcm);
  lua_pushinteger(L, result);
  return 1;
};

static int lua_lcm_destroy(lua_State *L)
{
  /* destroy lcm object */
  int i;
  lua_lcm_t *lcm = lua_checklcm(L, 1);
  if (lcm->lcm)
    lcm_destroy(lcm->lcm);
  for (i = 0; i < lcm->n_handlers; i++)
    luaL_unref(L, LUA_REGISTRYINDEX, lcm->handlers[i].reference);
  return 0;
};

static const struct luaL_reg lcm_methods[] = {
  {"get_fileno", lua_lcm_get_fileno},
  {"handle", lua_lcm_handle},
  {"__gc", lua_lcm_destroy},
  {NULL, NULL}
};

static const struct luaL_Reg lcm_functions[] = 
{
  {"new", lua_lcm_new},
  {NULL, NULL}
};

int luaopen_lcm(lua_State *L)
{
  luaL_newmetatable(L, "lcm_mt");
  lua_pushvalue(L, -1);
  lua_setfield(L, -2, "__index");
  luaL_register(L, NULL, lcm_methods);
  luaL_register(L, "lcm", lcm_functions);
  return 1;
}
