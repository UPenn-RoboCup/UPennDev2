#ifndef luaSaffirKinematics_H_DEFINED
#define luaSaffirKinematics_H_DEFINED

extern "C"
{
#include <lua.h>
#include <lualib.h>
#include <lauxlib.h>
}

extern "C"
int luaopen_SaffirKinematics(lua_State *L);

#endif
