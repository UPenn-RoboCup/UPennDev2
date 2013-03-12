#ifndef luaAtlasKinematics_H_DEFINED
#define luaAtlasKinematics_H_DEFINED

extern "C"
{
#include <lua.h>
#include <lualib.h>
#include <lauxlib.h>
}

extern "C"
int luaopen_AtlasKinematics(lua_State *L);

#endif
