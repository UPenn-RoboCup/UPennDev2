#ifndef lua_SPACENAV_H_DEFINED
#define lua_SPACENAV_H_DEFINED

extern "C"
{
#include "lua.h"
#include "lualib.h"
#include "lauxlib.h"
}

extern "C" int luaopen_Spacenav(lua_State *L);

#endif
