#include "lualcm.h"
#include "lua.h"

static int lua_lcm_new(lua_State *L)
{
  buf = (cbuffer *)lua_newuserdata(L, nbytes);

  if (str != NULL)
    memcpy(buf->data, str, len);

  buf->len = len;
  luaL_getmetatable(L, "lcm_mt");
  return 1;
};

static int lua_lcm_get_fileno(lua_State *L)
{

};

static int lua_lcm_subscribe(lua_State *L)
{

};

static int lua_lcm_publish(lua_State *L)
{

};

static int lua_lcm_handle(lua_State *L)
{

};

static int lua_subscription_set_queue_capacity(lua_State *L)
{

};

static int lua_lcm_destroy()
{

  return 1;
};
