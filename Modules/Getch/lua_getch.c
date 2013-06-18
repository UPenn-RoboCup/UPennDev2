/*
  Lua module to provide keyboard input
*/

#include <stdio.h>
#include <termios.h>
#include <fcntl.h>

#include "lua.h"
#include "lualib.h"
#include "lauxlib.h"


static int luagetch_block(lua_State *L){
  struct termios oldt, newt;
  int ch, oldf;

  tcgetattr(0, &oldt );
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(0, TCSANOW, &newt );
  oldf = fcntl(0, F_GETFL, 0);
  fcntl(0, F_SETFL, oldf & ~O_NONBLOCK);
  ch = getchar();
  tcsetattr(0, TCSANOW, &oldt );
  fcntl(0, F_SETFL, oldf);

  if (ch >= 0)
    lua_pushinteger(L, ch);
  else
    lua_pushnil(L);
  return 1;
}

static int luagetch_nonblock(lua_State *L) {
  struct termios oldt, newt;
  int ch, oldf;

  tcgetattr(0, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(0, TCSANOW, &newt);
  oldf = fcntl(0, F_GETFL, 0);
  fcntl(0, F_SETFL, oldf | O_NONBLOCK);
  ch = getchar();
  tcsetattr(0, TCSANOW, &oldt);
  fcntl(0, F_SETFL, oldf);

  if (ch >= 0)
    lua_pushinteger(L, ch);
  else
    lua_pushnil(L);
  return 1;
}

static const struct luaL_Reg getch_lib [] = {
  {"block", luagetch_block},
  {"nonblock", luagetch_nonblock},
  {NULL, NULL}
};

int luaopen_getch (lua_State *L) {
#if LUA_VERSION_NUM == 502
	  luaL_newlib(L, getch_lib);
#else
	luaL_register(L, "getch", getch_lib);
#endif
  return 1;
}
