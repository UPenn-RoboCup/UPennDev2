#ifndef luaTHOROPKinematics_H_DEFINED
#define luaTHOROPKinematics_H_DEFINED

extern "C"
{
#include <lua.h>
#include <lualib.h>
#include <lauxlib.h>
}

extern "C"
int luaopen_THOROPKinematics(lua_State *L);

#endif
