#ifndef luaHokuyo_h_DEFINED
#define luaHokuyo_H_DEFINED

extern "C"
{
#include "lua.h"
#include "lualib.h"
#include "lauxlib.h"
}

extern "C"
int luaopen_Hokuyo(lua_State *L);

#endif
