#ifndef luaCharliKinematics_H_DEFINED
#define luaCharliKinematics_H_DEFINED

extern "C"
{
#include <lua.h>
#include <lualib.h>
#include <lauxlib.h>
}

extern "C"
int luaopen_CharliKinematics(lua_State *L);

#endif
