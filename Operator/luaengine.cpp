#include "luaengine.h"
#include <iostream>

extern "C"{
#include "lua.h"
#include "lualib.h"
#include "lauxlib.h"
}

LuaEngine::LuaEngine(QObject *parent) :
    QObject(parent)
{
}

/*int my_function(lua_State *L)
{
  int argc = lua_gettop(L);

  std::cerr << "-- my_function() called with " << argc
    << " arguments:" << std::endl;

  for ( int n=1; n<=argc; ++n ) {
    std::cerr << "-- argument " << n << ": "
      << lua_tostring(L, n) << std::endl;
  }

  lua_pushnumber(L, 123); // return value
  return 1; // number of return values
}

void report_errors(lua_State *L, int status)
{
  if ( status!=0 ) {
    std::cerr << "-- " << lua_tostring(L, -1) << std::endl;
    lua_pop(L, 1); // remove error message
  }
}

int LuaEngine::runLua(const std::string& filename)
{
    lua_State *L = lua_open();

    luaopen_io(L); // provides io.*
    luaopen_base(L);
    luaopen_table(L);
    luaopen_string(L);
    luaopen_math(L);
    //luaopen_loadlib(L);

    // make my_function() available to Lua programs
    lua_register(L, "my_function", my_function);

    std::cerr << "-- Loading file: " << filename.c_str() << std::endl;

    int s = luaL_loadfile(L, filename.c_str());

    if ( s==0 ) {
      // execute Lua program
      s = lua_pcall(L, 0, LUA_MULTRET, 0);
    }

    report_errors(L, s);
    lua_close(L);
    std::cerr << std::endl;
  return 0;

}*/
