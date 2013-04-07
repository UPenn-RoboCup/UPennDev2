#include "config.h"

// config : global interface for config data
// author : Mike Hopkins
///////////////////////////////////////////////////////////////////////////

static std::vector<int> lua_get_int_vector(lua_State *L, int index);
static std::vector<bool> lua_get_boolean_vector(lua_State *L, int index);
static std::vector<double> lua_get_double_vector(lua_State *L, int index);
static std::vector<std::string> lua_get_string_vector(lua_State *L, int index);

Config::Config()
{
  // load config values from Config.lua module
  L = luaL_newstate();
  luaL_openlibs(L);
  if (luaL_loadfile(L, "Run/include.lua") || lua_pcall(L, 0, 0, 0))
  {
    fprintf(stderr, "(%s)\n", lua_tostring(L, -1));
    fflush( stderr );
    exit(EXIT_FAILURE);
  }
  if (luaL_loadstring(L, "require('Params')") || lua_pcall(L, 0, 0, 0))
  {
    fprintf(stderr, "(%s)", lua_tostring(L, -1));
    fflush( stderr );
    exit(EXIT_FAILURE);
  }
  lua_getglobal(L, "Params");
  if (!lua_istable(L, -1))
  {
    fprintf(stderr, "Unable to load Config table from Params.lua!\n");
    fflush( stderr );
    exit(EXIT_FAILURE);
  }
  lua_getfield(L, -1, "get_field");
  if (!lua_isfunction(L, -1))
  {
    fprintf(stderr, "unable to load Config.get_field() method");
    fflush( stderr );
    exit(EXIT_FAILURE);
  }
  lua_pop(L, 2);
}

Config::~Config()
{
  lua_close(L);
}

int Config::get_int(std::string field)
{
  push_field(field);
  int result = (int)luaL_checknumber(L, -1);
  lua_pop(L, 1);
  return result;
}

bool Config::get_boolean(std::string field)
{
  push_field(field);
  bool result = lua_toboolean(L, -1);
  lua_pop(L, 1);
  return result;
}

double Config::get_double(std::string field)
{
  push_field(field);
  double result = luaL_checknumber(L, -1);
  lua_pop(L, 1);
  return result;
}

std::string Config::get_string(std::string field)
{
  size_t len;
  push_field(field);
  const char *str = luaL_checklstring(L, -1, &len);
  std::string result(str, len);
  lua_pop(L, 1);
  return result;
}

std::vector<int> Config::get_int_vector(std::string field)
{
  push_field(field);
  std::vector<int> result = lua_get_int_vector(L, -1);
  return result;
}

std::vector<bool> Config::get_boolean_vector(std::string field)
{
  push_field(field);
  std::vector<bool> result = lua_get_boolean_vector(L, -1);
  return result;
}

std::vector<double> Config::get_double_vector(std::string field)
{
  push_field(field);
  std::vector<double> result = lua_get_double_vector(L, -1);
  return result;
}

std::vector<std::string> Config::get_string_vector(std::string field)
{
  push_field(field);
  std::vector<std::string> result = lua_get_string_vector(L, -1);
  return result;
}

void Config::push_field(std::string field)
{
  lua_getglobal(L, "Params");
  lua_getfield(L, -1, "get_field");
  if (lua_isfunction(L, -1))
  {
    lua_pushstring(L, field.c_str());
    lua_call(L, 1, 1);
  }
  lua_remove(L, -2);
}

std::vector<int> lua_get_int_vector(lua_State *L, int index)
{
  // get double vector from lua stack
  std::vector<int> vec(0);
  if (!lua_istable(L, index))
    return vec;
  size_t len = lua_objlen(L, index);
  vec.resize(len);
  if (index < 0)
    --index;

  for (int i = 0; i < len; i++)
  {
    lua_pushinteger(L, i + 1);
    lua_gettable(L, index);
    vec[i] = (int)luaL_checknumber(L, -1);
    lua_pop(L, 1);
  }
  return vec;
}

std::vector<bool> lua_get_boolean_vector(lua_State *L, int index)
{
  // get bool vector from lua stack
  std::vector<bool> vec(0);
  if (!lua_istable(L, index))
    return vec;
  size_t len = lua_objlen(L, index);
  vec.resize(len);
  if (index < 0)
    --index;

  for (int i = 0; i < len; i++)
  {
    lua_pushinteger(L, i + 1);
    lua_gettable(L, index);
    vec[i] = lua_toboolean(L, -1);
    lua_pop(L, 1);
  }
  return vec;
}

std::vector<double> lua_get_double_vector(lua_State *L, int index)
{
  // get double vector from lua stack
  std::vector<double> vec(0);
  if (!lua_istable(L, index))
    return vec;
  size_t len = lua_objlen(L, index);
  vec.resize(len);
  if (index < 0)
    --index;

  for (int i = 0; i < len; i++)
  {
    lua_pushinteger(L, i + 1);
    lua_gettable(L, index);
    vec[i] = luaL_checknumber(L, -1);
    lua_pop(L, 1);
  }
  return vec;
}

std::vector<std::string> lua_get_string_vector(lua_State *L, int index)
{
  // get double vector from lua stack
  std::vector<std::string> vec(0);
  if (!lua_istable(L, index)){
    fprintf(stdout,"Not a table!\n");
    fflush(stdout);
    return vec;
  }
  size_t len = lua_objlen(L, index);
  vec.resize(len);
  if (index < 0)
    --index;

  for (int i = 0; i < len; i++)
  {
    lua_pushinteger(L, i + 1);
    lua_gettable(L, index);
    size_t len;
    const char *str = luaL_checklstring(L, -1, &len);
    vec[i] = std::string(str, len);
    lua_pop(L, 1);
  }
  return vec;
}
