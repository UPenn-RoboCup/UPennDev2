#ifndef LUA_KINEMATICS_H 
#define LUA_KINEMATICS_H 

extern "C"
{
#include <lua.h>
#include <lualib.h>
#include <lauxlib.h>
}

extern "C"
int luaopen_Kinematics(lua_State *L);

#endif
