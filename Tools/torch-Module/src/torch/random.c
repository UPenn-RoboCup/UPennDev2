#include "luaT.h"
#include "TH.h"
   
static int wrapper_seed(lua_State *L)
{
int narg = lua_gettop(L);
long arg1 = 0;
if(narg == 0
)
{
}
else
luaL_error(L, "expected arguments: ");
arg1 = THRandom_seed();
lua_pushnumber(L, (lua_Number)arg1);
return 1;
}

static int wrapper_initialSeed(lua_State *L)
{
int narg = lua_gettop(L);
long arg1 = 0;
if(narg == 0
)
{
}
else
luaL_error(L, "expected arguments: ");
arg1 = THRandom_initialSeed();
lua_pushnumber(L, (lua_Number)arg1);
return 1;
}

static int wrapper_manualSeed(lua_State *L)
{
int narg = lua_gettop(L);
long arg1 = 0;
if(narg == 1
&& lua_isnumber(L, 1)
)
{
arg1 = (long)lua_tonumber(L, 1);
}
else
luaL_error(L, "expected arguments: long");
THRandom_manualSeed(arg1);
return 0;
}

static const struct luaL_Reg random__ [] = {
{"seed", wrapper_seed},
{"initialSeed", wrapper_initialSeed},
{"manualSeed", wrapper_manualSeed},
{NULL, NULL}
};

void torch_random_init(lua_State *L)
{
  luaL_register(L, NULL, random__);
}
   