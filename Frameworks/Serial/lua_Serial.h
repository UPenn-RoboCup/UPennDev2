#ifndef lua_Serial_h_DEFINED
#define lua_Serial_H_DEFINED

#ifdef __cplusplus
extern "C"
{
#endif

#include "lua.h"
#include "lualib.h"
#include "lauxlib.h"

#ifdef __cplusplus
}
#endif

int luaopen_Serial(lua_State *L);

#endif
