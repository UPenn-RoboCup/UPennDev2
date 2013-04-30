#ifndef luaOctomap_h_DEFINED
#define luaOctomap_H_DEFINED

extern "C"
{
#include "lua.h"
#include "lualib.h"
#include "lauxlib.h"
#include "torch/luaT.h"
#include "torch/TH/TH.h"
}

#include <stdio.h>
#include <vector>
#include <string>

#include <octomap/octomap.h>
#include <octomap/math/Utils.h>

extern "C"
int luaopen_Octomap(lua_State *L);

#endif
