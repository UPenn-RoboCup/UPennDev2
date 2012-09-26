/*
 * Lua module to access c datatypes embedded in strings
 * Author: Mike Hopkins
 */

#include <stdint.h>
#include <string.h>
#include "luacbuffer.h"

#define STR2(a, b) (((a) << 8) + (b))

typedef struct cbuffer {
  size_t len;
  uint8_t data[1];
} cbuffer;

static int lua_cbuffer_to_string(lua_State *L)
{
  cbuffer *buf = lua_checkcbuffer(L, 1);
  int offset = luaL_optinteger(L, 2, 0);
  if (offset > buf->len - 1)
    luaL_error(L, "cbuffer: Out of bounds memory access");
  lua_pushlstring(L, (char *)(buf->data + offset), buf->len - offset);
  return 1;
}

static int lua_cbuffer_new(lua_State *L)
{
  cbuffer *buf;
  const char *str = NULL;
  uint8_t datatype[16];
  size_t len, nbytes;

  if (lua_type(L, 1) == LUA_TSTRING) {
    /* initialize cbuffer with string */
    str = lua_tolstring(L, 1, &len);
  }
  else {
    /* specify cbuffer length in bytes */
    len = lua_tonumber(L, 1);
  }

  nbytes = sizeof(cbuffer) + len - 1;
  buf = (cbuffer *)lua_newuserdata(L, nbytes);
  if (str != NULL)
    memcpy(buf->data, str, len);
  buf->len = len;

  luaL_getmetatable(L, "cbuffer_mt");
  lua_setmetatable(L, -2);
  return 1;
}

static int lua_cbuffer_set(lua_State *L)
{
  uint8_t datatype[16];
  uint8_t *ptr = datatype; 
  size_t n = 0, len = 1;

  cbuffer *buf = lua_checkcbuffer(L, 1);
  const char *type = luaL_checklstring(L, 2, &n);
  const char *str = lua_tolstring(L, 3, &len);
  double num = lua_tonumber(L, 3);
  int offset = luaL_optinteger(L, 4, 0);

  if (!n)
    luaL_error(L, "cbuffer: invalid type");

  switch (STR2(type[0], type[n-1])) {
    case STR2('s','g'):
      ptr = (uint8_t *)str;
      break;
    case STR2('i','8'):
      len = sizeof(int8_t);
      *(int8_t *)ptr = (int8_t)num;
      break;
    case STR2('u','8'):
      len = sizeof(uint8_t);
      *(uint8_t *)ptr = (uint8_t)num;
      break;
    case STR2('i','6'):
      len = sizeof(int16_t);
      *(int16_t *)ptr = (int16_t)num;
      break;
    case STR2('u','6'):
      len = sizeof(uint16_t);
      *(uint16_t *)ptr = (uint16_t)num;
      break;
    case STR2('i','2'):
      len = sizeof(int32_t);
      *(int32_t *)ptr = (int32_t)num;
      break;
    case STR2('u','2'):
      len = sizeof(uint32_t);
      *(uint32_t *)ptr = (uint32_t)num;
      break;
    case STR2('f','t'):
      len = sizeof(float);
      *(float *)ptr = (float)num;
      break;
    case STR2('d','e'):
      len = sizeof(double);
      *(double *)ptr = (double)num;
      break;
    default:
      luaL_error(L, "cbuffer: invalid type");
  }

  if (offset > (buf->len - len))
    luaL_error(L, "cbuffer: out of bounds memory access");
  memcpy(buf->data + offset, ptr, len);
  return 0;
}

static int lua_cbuffer_get(lua_State *L)
{
  size_t n = 0; 
  cbuffer *buf = lua_checkcbuffer(L, 1);
  const char *type = luaL_checklstring(L, 2, &n);
  int offset = luaL_optinteger(L, 3, 0);
  void *ptr = (void *)(buf->data + offset);

  if (!n)
    luaL_error(L, "cbuffer: invalid type");

  switch (STR2(type[0], type[n-1])) {
    case STR2('s','g'):
      lua_remove(L, 2);
      return lua_cbuffer_to_string(L);
      break;
    case STR2('i','8'):
      if (offset > (buf->len - sizeof(int8_t)))
        luaL_error(L, "cbuffer: out of bounds memory access");
      lua_pushnumber(L, *(int8_t *)ptr);
      break;
    case STR2('u','8'):
      if (offset > (buf->len - sizeof(uint8_t)))
        luaL_error(L, "cbuffer: out of bounds memory access"); 
      lua_pushnumber(L, *(uint8_t *)ptr);
      break;
    case STR2('i','6'):
      if (offset > (buf->len - sizeof(int16_t)))
        luaL_error(L, "cbuffer: out of bounds memory access");
      lua_pushnumber(L, *(int16_t *)ptr);
      break;
    case STR2('u','6'):
      if (offset > (buf->len - sizeof(uint16_t)))
        luaL_error(L, "cbuffer: out of bounds memory access"); 
      lua_pushnumber(L, *(uint16_t *)ptr);
      break;
    case STR2('i','2'):
      if (offset > (buf->len - sizeof(int32_t)))
        luaL_error(L, "cbuffer: out of bounds memory access"); 
      lua_pushnumber(L, *(int32_t *)ptr);
      break;
    case STR2('u','2'):
      if (offset > (buf->len - sizeof(uint32_t)))
        luaL_error(L, "cbuffer: out of bo6unds memory access"); 
      lua_pushnumber(L, *(uint32_t *)ptr);
      break;
    case STR2('f','t'):
      if (offset > (buf->len - sizeof(float)))
        luaL_error(L, "cbuffer: out of bounds memory access"); 
      lua_pushnumber(L, *(float *)ptr);
      break;
    case STR2('d','e'):
      if (offset > (buf->len - sizeof(double)))                  
        luaL_error(L, "cbuffer: out of bounds memory access"); 
      lua_pushnumber(L, *(double *)ptr);
      break;
    default:
      luaL_error(L, "cbuffer: invalid type");
  }
  return 1;
}

static int lua_cbuffer_sizeof(lua_State *L)
{
  size_t n = 0;
  int size = 0;
  const char* type = luaL_checklstring(L, 1, &n);

  if (!n)
    luaL_error(L, "cbuffer: invalid type");

  switch (STR2(type[0], type[n-1])) {
    case STR2('i','8'):
      size = sizeof(int8_t);
      break;
    case STR2('u','8'):
      size = sizeof(uint8_t);
      break;
    case STR2('i','6'):
      size = sizeof(int16_t);
      break;
    case STR2('u','6'):
      size = sizeof(uint16_t);
      break;
    case STR2('i','2'):
      size = sizeof(int32_t);
      break;
    case STR2('u','2'):
      size = sizeof(uint32_t);
      break;
    case STR2('f','t'):
      size = sizeof(float);
      break;
    case STR2('d','e'):
      size = sizeof(double);
      break;
    default:
      luaL_error(L, "cbuffer: invalid type");
  }
  lua_pushinteger(L, size);
  return 1;
}

static const struct luaL_Reg cbuffer_methods [] =
{
  {"get", lua_cbuffer_get},
  {"set", lua_cbuffer_set},
  {"__tostring", lua_cbuffer_to_string},
  {NULL, NULL}
};

static const struct luaL_Reg cbuffer_functions [] = 
{
  {"new", lua_cbuffer_new},
  {"sizeof", lua_cbuffer_sizeof},
  {NULL, NULL}
};

int luaopen_cbuffer(lua_State *L)
{
  luaL_newmetatable(L, "cbuffer_mt");
  lua_pushvalue(L, -1);
  lua_setfield(L, -2, "__index");
  luaL_register(L, NULL, cbuffer_methods);
  luaL_register(L, "cbuffer", cbuffer_functions);
  return 1;
}
