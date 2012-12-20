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

#include "kBotPacket2.h"
#include <map>
#include <string.h>
#include <stdint.h>
#include <iostream>

using namespace std;

std::map<int,kBotPacket2> packets;
static int packetCntr = 0;

static int lua_kBPacket_create(lua_State *L) {
    packets[packetCntr] = kBotPacket2();
    kBotPacket2Init(&(packets[packetCntr]));
    lua_pushinteger(L, packetCntr);
    packetCntr++;
    return 1;
}

static int lua_kBPacket_processBuffer(lua_State *L) {
    if (lua_isnoneornil(L, 3)) 
      luaL_error(L, "need three input arguments");

    int nPacket = luaL_checkint(L, 1);
    if (nPacket >= packetCntr)
      luaL_error(L, "invalid packet counter");

    uint8_t *buf = (uint8_t *) lua_touserdata(L, 2); 
    if ((buf == NULL) || !lua_islightuserdata(L, 2)) {
      return luaL_error(L, "Input string not light user data");
    }
    int size = luaL_checkint(L, 3);


    kBotPacket2 * packet = &(packets[nPacket]);
    while (size > 0)
    {
      int ret = kBotPacket2ProcessChar(*buf++,packet);
      size--;
      if (ret > 0)
      {
        if (packet->buffer[4] == 34) {
          for (int i = 0; i < packet->lenExpected; i++)
            cout << (unsigned int)packet->buffer[i] << ' ';
          cout << endl;
          lua_pushlightuserdata(L, packet->buffer);
          lua_pushstring(L, "uint8_t");
          lua_pushinteger(L, packet->lenExpected);
  
          lua_pushlightuserdata(L, buf);
          lua_pushstring(L, "uint8_t");
          lua_pushinteger(L, size);
          
          return 6;

        }
      }
    }

    lua_pushinteger(L, 1);
    lua_pushinteger(L, 1);
    
    return 1;
}

static const struct luaL_reg kBPacket_lib [] = {
  {"create", lua_kBPacket_create},
  {"processBuffer", lua_kBPacket_processBuffer},
  {NULL, NULL}
};

extern "C"
int luaopen_kBPacket(lua_State *L) {
  luaL_register(L, "kBPacket", kBPacket_lib);
  return 1;
}


