/*
  Lua module to provide process dynamixel packets
*/

#include "dynamixel1.h"
#include "lua.h"
#include "lualib.h"
#include "lauxlib.h"

const char* errtable[] = {
"Input Voltage Error",
"Angle Limit Error",
"Overheating Error",
"Range Error",
"Checksum Error",
"Overload Error",
"Instruction Error",
"Motor status Error"
};

// Error decoder
static int lua_dynamixel_error(lua_State *L) {
	size_t nerrbits, i;
	const char *errbits_ptr = luaL_checklstring(L, 1, &nerrbits);
	// TODO: check that only one byte was given
	char errbits = *errbits_ptr;
	uint8_t errmask = 0x01, nerr = 0;
	
	lua_newtable(L);
	for (i = 0; i < 8; i++) {
		if (errbits & errmask){
			lua_pushstring(L, errtable[i]);
			lua_rawseti(L, -2, ++nerr);
			//printf("%d: %d|%d (%.2X): %s\n",errbits,i,nerr,errmask,errtable[i]);
		}
		errmask<<=1;
	} //for
	return nerr;
}

static int lua_pushpacket(lua_State *L, DynamixelPacket *p) {
  if (p != NULL) {
    lua_pushlstring(L, (char *)p, p->length + 4);
    return 1;
  }
  return 0;
}

static int lua_dynamixel_instruction_ping(lua_State *L) {
  int id = luaL_checkint(L, 1);
  DynamixelPacket *p = dynamixel_instruction_ping(id);
  return lua_pushpacket(L, p);
}

static int lua_dynamixel_instruction_read_data(lua_State *L) {
  int id = luaL_checkint(L, 1);
  unsigned char addr = luaL_checkint(L, 2);
  unsigned char len = luaL_optinteger(L, 3, 1);
  DynamixelPacket *p = dynamixel_instruction_read_data
    (id, addr, len);
  return lua_pushpacket(L, p);
}

//ADDED for bulk read
static int lua_dynamixel_instruction_bulk_read_data(lua_State *L) {
  uchar id_cm730 = luaL_checkint(L, 1);
  size_t nstr;
  const char *str = luaL_checklstring(L, 2, &nstr);
  uchar addr = luaL_checkint(L, 3);
  uchar len = luaL_checkint(L, 4);
  DynamixelPacket *p = dynamixel_instruction_bulk_read_data
    (id_cm730, (uchar *) str, addr, len, nstr);
  return lua_pushpacket(L, p);
}


static int lua_dynamixel_instruction_write_data(lua_State *L) {
  uchar id = luaL_checkint(L, 1);
  uchar addr = luaL_checkint(L, 2);
  size_t nstr;
  const char *str = luaL_checklstring(L, 3, &nstr);
  DynamixelPacket *p = dynamixel_instruction_write_data
    (id, addr, (uchar *)str, nstr);
  return lua_pushpacket(L, p);
}

static int lua_dynamixel_instruction_write_byte(lua_State *L) {
  uchar id = luaL_checkint(L, 1);
  uchar addr = luaL_checkint(L, 2);
  uchar byte = luaL_checkint(L, 3);
  DynamixelPacket *p = dynamixel_instruction_write_data
    (id, addr, &byte, 1);
  return lua_pushpacket(L, p);
}

static int lua_dynamixel_instruction_write_word(lua_State *L) {
  uchar id = luaL_checkint(L, 1);
  uchar addr = luaL_checkint(L, 2);
  unsigned short word = luaL_checkint(L, 3);
  uchar byte[2];
  byte[0] = (word & 0x00FF);
  byte[1] = (word & 0xFF00) >> 8;
  DynamixelPacket *p = dynamixel_instruction_write_data
    (id, addr, byte, 2);
  return lua_pushpacket(L, p);
}

static int lua_dynamixel_instruction_sync_write(lua_State *L) {
  uchar addr = luaL_checkint(L, 1);
  uchar len = luaL_checkint(L, 2);
  size_t nstr;
  const char *str = luaL_checklstring(L, 3, &nstr);
  DynamixelPacket *p = dynamixel_instruction_sync_write
    (addr, len, (uchar *)str, nstr);
  return lua_pushpacket(L, p);
}

static int lua_dynamixel_input(lua_State *L) {
  size_t nstr;
  const char *str = luaL_checklstring(L, 1, &nstr);
  int nPacket = luaL_optinteger(L, 2, 1)-1;

  DynamixelPacket pkt;
  int ret = 0, offset = 0, i;

  /* Packet Table */
  lua_newtable(L);
  for (i = 0; i < nstr; i++) {
    /* Try to find a packet */
    nPacket = dynamixel_input(&pkt, str[i], nPacket);
    if (nPacket < 0){
      /* Packet found */
      ret += lua_pushpacket(L, &pkt);
      /* Index this in the table */
      lua_rawseti(L, -2, ret);
      offset = i+1;
    }
  } /* for */

  /* Push the leftover string */
  lua_pushlstring(L, str+offset, nstr-offset );
  return 2;
}

static int lua_dynamixel_byte_to_word(lua_State *L) {
  int n = lua_gettop(L), ret = 0, i;
  for (i = 1; i < n; i += 2) {
    unsigned short byteLow = luaL_checkint(L, i);
    unsigned short byteHigh = luaL_checkint(L, i+1);
    unsigned short word = (byteHigh << 8) + byteLow;
    lua_pushnumber(L, word);
    ret++;
  }
  return ret;
}

static int lua_dynamixel_word_to_byte(lua_State *L) {
  int n = lua_gettop(L), ret = 0, i;
  for (i = 1; i <= n; i++) {
    unsigned short word = luaL_checkint(L, i);
    unsigned short byteLow = word & 0x00FF;
    lua_pushnumber(L, byteLow);
    ret++;
    unsigned short byteHigh = (word & 0xFF00)>>8;
    lua_pushnumber(L, byteHigh);
    ret++;
  }
  return ret;
}

static const struct luaL_Reg dynamixelpacket_functions[] = {
  {"input", lua_dynamixel_input},
  {"ping", lua_dynamixel_instruction_ping},
  {"write_data", lua_dynamixel_instruction_write_data},
  {"write_byte", lua_dynamixel_instruction_write_byte},
  {"write_word", lua_dynamixel_instruction_write_word},
  {"sync_write", lua_dynamixel_instruction_sync_write},
  {"read_data", lua_dynamixel_instruction_read_data},
  {"bulk_read_data", lua_dynamixel_instruction_bulk_read_data},
  {"word_to_byte", lua_dynamixel_word_to_byte},
  {"byte_to_word", lua_dynamixel_byte_to_word},
	{"strerr",lua_dynamixel_error},
  {NULL, NULL}
};

#if LUA_VERSION_NUM != 502
static const struct luaL_Reg dynamixelpacket_methods[] = {
  {NULL, NULL}
};
#endif

#ifdef __cplusplus
extern "C"
#endif
int luaopen_DynamixelPacket1 (lua_State *L) {

#if LUA_VERSION_NUM == 502
  luaL_newlib(L, dynamixelpacket_functions);
#else
  // OO access: mt.__index = mt
  // Not compatible with array access
  luaL_newmetatable(L, "dynamixelpacket_mt");
  lua_pushvalue(L, -1);
  lua_setfield(L, -2, "__index");
  luaL_register(L, NULL, dynamixelpacket_methods);
  luaL_register(L, "DynamixelPacket1", dynamixelpacket_functions);
#endif

  return 1;
}
