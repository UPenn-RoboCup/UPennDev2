#ifndef luaOctomap_h_DEFINED
#define luaOctomap_H_DEFINED

extern "C"
{
#include "lua.h"
#include "lualib.h"
#include "lauxlib.h"
}

#include <stdio.h>
#include <vector>
#include <string>

extern "C"
int luaopen_Octomap(lua_State *L);

#endif
