#ifndef luaBALLFILTER_H_DEFINED
#define luaBALLFILTER_H_DEFINED

#include "BallModel.h"
#include <math.h>

extern "C"
{
#include "lua.h"
#include "lualib.h"
#include "lauxlib.h"
}

extern "C"
int luaopen_ballfilter(lua_State *L);

#endif
