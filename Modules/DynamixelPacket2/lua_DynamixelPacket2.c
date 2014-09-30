/*
Lua module to provide process dynamixel packets
Daniel D. Lee copyright 2010 <ddlee@seas.upenn.edu>
Stephen G. McGill copyright 2013 <smcgill3@seas.upenn.edu>
*/
#include "lua.h"
#include "lualib.h"
#include "lauxlib.h"
#include "dynamixel2.h"

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
  size_t nerrbits;
  const char *errbits_ptr = luaL_checklstring(L, 1, &nerrbits);
  // TODO: check that only one byte was given
  //uint8_t errbits = (uint8_t)(*errbits_ptr);
  char errbits = *errbits_ptr;
  uint8_t errmask = 0x01, nerr = 0, i;
  
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

static int lua_crc16(lua_State *L) {
  size_t nstr;
  const unsigned char *str = (unsigned char *)luaL_checklstring(L, 1, &nstr);
  uint16_t crc = dynamixel_crc(0, str, nstr);
  lua_pushnumber( L, crc & 0x00FF ); // low
  lua_pushnumber( L, (crc>>8) & 0x00FF ); // high
  return 2;
}

static int lua_pushpacket(lua_State *L, DynamixelPacket *p) {
  if (p != NULL) {
    lua_pushlstring(L, (char *)p, p->length + 7);
    return 1;
  }
  return 0;
}

static int lua_dynamixel_instruction_ping(lua_State *L) {
  int id = luaL_checkint(L, 1);
  DynamixelPacket *p = dynamixel_instruction_ping(id);
  return lua_pushpacket(L, p);
}

static int lua_dynamixel_instruction_write_data(lua_State *L) {
  uint8_t id = luaL_checkint(L, 1);
  size_t naddr;
  const char *addr = luaL_checklstring(L, 2, &naddr);
  size_t nstr;
  const char *str = luaL_checklstring(L, 3, &nstr);
  DynamixelPacket *p = dynamixel_instruction_write_data
    (id, addr[0], addr[1], (uint8_t *)str, nstr);
  return lua_pushpacket(L, p);
}

static int lua_dynamixel_instruction_write_byte(lua_State *L) {
  uint8_t id = luaL_checkint(L, 1);
  size_t naddr;
  const char *addr = luaL_checklstring(L, 2, &naddr);
  uint8_t byte = luaL_checkint(L, 3);
  DynamixelPacket *p = dynamixel_instruction_write_data
    (id, addr[0], addr[1], &byte, 1); //TODO: endianness?????
  return lua_pushpacket(L, p);
}

static int lua_dynamixel_instruction_write_word(lua_State *L) {
  uint8_t id = luaL_checkint(L, 1);
  size_t naddr;
  const char *addr = luaL_checklstring(L, 2, &naddr);
  uint16_t word = luaL_checkint(L, 3);
  uint8_t byte[2];
  byte[0] = (word & 0x00FF);
  byte[1] = (word>>8) & 0x00FF;
  DynamixelPacket *p = dynamixel_instruction_write_data
    (id, addr[0], addr[1], byte, 2);
  return lua_pushpacket(L, p);
}

static int lua_dynamixel_instruction_write_dword(lua_State *L) {
  uint8_t id = luaL_checkint(L, 1);
  size_t naddr;
  const char *addr = luaL_checklstring(L, 2, &naddr);
  // TODO: Check that the addr is always of length 2
  int32_t dword = luaL_checkint(L, 3);
//  fprintf(stdout,"DWORD: %X\n",dword);
//  fflush(stdout);

  uint8_t byte[4];
  byte[0] = (dword & 0x000000FF);
  byte[1] = (dword & 0x0000FF00)>>8;
  byte[2] = (dword & 0x00FF0000)>>16;
  byte[3] = (dword>>24) & 0x000000FF;
  DynamixelPacket *p = dynamixel_instruction_write_data
    (id, addr[0], addr[1], byte, 4);
  return lua_pushpacket(L, p);
}

static int lua_dynamixel_instruction_sync_write(lua_State *L) {
  size_t naddr;
  const char *addr = luaL_checklstring(L, 1, &naddr);
  uint16_t len  = luaL_checkint(L, 2);
  size_t nstr;
  const char *str = luaL_checklstring(L, 3, &nstr);
  DynamixelPacket *p = dynamixel_instruction_sync_write
    (addr[0], addr[1], len, (uint8_t *)str, nstr);
  return lua_pushpacket(L, p);
}

static int lua_dynamixel_instruction_bulk_write(lua_State *L) {
  // How many IDs to read from
  size_t nids, naddr;
	int i;
	uint8_t id;
  // Which ids to read from
  const char *ids = luaL_checklstring(L, 1, &nids);
	// Initialize the header
	dynamixel_instruction_init_bulk_write();
	const char *addr;
	uint16_t addr_len;
	// Table of addr/len pairs
  luaL_checktype(L, 2, LUA_TTABLE);
	// Table of values
	luaL_checktype(L, 3, LUA_TTABLE);
	for (i = 1; i<=nids; i++) {
		id = ids[i-1];
    // first, put the table element on the top of the stack
    lua_rawgeti(L, 2, i);
		// Get the bulk elements
		// Address is the first element
		lua_rawgeti(L, -1, 1);
		addr = luaL_checklstring(L, -1, &naddr);
		lua_pop(L, 1);
		// How many bytes to read is the second element
		lua_rawgeti(L, -1, 2);
		addr_len = luaL_checkint(L, -1);
		lua_pop(L, 1);
		// Get the values from table at 3
		lua_rawgeti(L, 3, i);
		int32_t value = luaL_checknumber(L, -1);
		lua_pop(L, 1);
		dynamixel_instruction_add_bulk_write(id, addr[0], addr[1], addr_len, value);
	}

	DynamixelPacket *p = dynamixel_instruction_finalize_bulk_write();

  return lua_pushpacket(L, p);
}

static int lua_dynamixel_instruction_read_data(lua_State *L) {
  int id = luaL_checkint(L, 1);
  size_t naddr;
  const char *addr = luaL_checklstring(L, 2, &naddr);
  unsigned char len = luaL_optinteger(L, 3, 1);
  DynamixelPacket *p = dynamixel_instruction_read_data
    (id, addr[0], addr[1], len);
  return lua_pushpacket(L, p);
}

static int lua_dynamixel_instruction_sync_read(lua_State *L) {
  // How many IDs to read from
  size_t nids;
  // Which ids to read from
  const char *ids = luaL_checklstring(L, 1, &nids);
  // TODO: Verify naddr=2 for 2-byte address
  size_t naddr;
  const char *addr = luaL_checklstring(L, 2, &naddr);
  // How many bytes to read
  uint16_t len = luaL_checkint(L, 3);
  DynamixelPacket *p = dynamixel_instruction_sync_read
    (addr[0], addr[1], len, (uint8_t *)ids, (uint8_t)nids);
  return lua_pushpacket(L, p);
}

static int lua_dynamixel_instruction_bulk_read(lua_State *L) {
  // How many IDs to read from
  size_t nids, naddr;
	int i;
	uint8_t id;
  // Which ids to read from
  const char *ids = luaL_checklstring(L, 1, &nids);
	// Initialize the header
	dynamixel_instruction_init_bulk_read();
	const char *addr;
	uint16_t addr_len;
	// Make sure we are dealing with a table of values
  luaL_checktype(L, 2, LUA_TTABLE);
	for (i = 1; i<=nids; i++) {
		id = ids[i-1];
    // first, put the table element on the top of the stack
    lua_rawgeti(L, 2, i);
		// Get the bulk elements
		// Address is the first element
		lua_rawgeti(L, -1, 1);
		addr = luaL_checklstring(L, -1, &naddr);
		lua_pop(L, 1);
		// How many bytes to read is the second element
		lua_rawgeti(L, -1, 2);
		addr_len = luaL_checkint(L, -1);
		lua_pop(L, 1);
		dynamixel_instruction_add_bulk_read(id, addr[0], addr[1], addr_len);
	}

	DynamixelPacket *p = dynamixel_instruction_finalize_bulk_read();

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

static int lua_dynamixel_word_to_byte(lua_State *L) {
  int n = lua_gettop(L), ret = 0, i;
  int16_t word;
  uint16_t byteLow, byteHigh;
  for (i = 1; i <= n; i++) {
    word = luaL_checkint(L, i);
    byteLow  = (word & 0x00FF);
    byteHigh = ((uint16_t)(word & 0xFF00))>>8;
    lua_pushnumber(L, byteLow);
    lua_pushnumber(L, byteHigh);
    ret+=2;
  }
  return ret;
}

static int lua_dynamixel_dword_to_byte(lua_State *L) {
  int n = lua_gettop(L), ret = 0, i;
  int32_t dword;
  uint32_t byteLow, byteHigh, byteHigher, byteHighest;
  for (i = 1; i <= n; i++) {
    dword = luaL_checkint(L, i);
    //printf("DWORD | %d: %16.8X\n", dword, dword);
    byteLow     = (dword & 0x000000FF);
    byteHigh    = ((uint32_t)(dword & 0x0000FF00))>>8;
    byteHigher  = ((uint32_t)(dword & 0x00FF0000))>>16;
    byteHighest = (((uint32_t)(dword & 0xFF000000))>>24) & 0xFF;
    lua_pushnumber(L, byteLow);
    lua_pushnumber(L, byteHigh);
    lua_pushnumber(L, byteHigher);
    lua_pushnumber(L, byteHighest);
    ret+=4;
  }
  return ret;
}

static int lua_dynamixel_byte_to_word(lua_State *L) {
  int n = lua_gettop(L), ret = 0, i;
  int16_t word;
  uint16_t byteLow, byteHigh;
  for (i = 1; i < n; i += 2) {
    byteLow = luaL_checkint(L, i);
    byteHigh = luaL_checkint(L, i+1);
    word = (int16_t)((byteHigh<<8)&0xFF00) + (int16_t)byteLow;
    // All 2 byte registers are signed numbers
    lua_pushnumber(L, word);
    ret++;
  }
  return ret;
}

static int lua_dynamixel_byte_to_dword(lua_State *L) {
  int n = lua_gettop(L), ret = 0, i;
  uint32_t dword, byteLow, byteHigh, byteHigher, byteHighest;
  for (i = 1; i < n; i += 4) {
    byteLow = luaL_checkint(L, i);
    byteHigh = luaL_checkint(L, i+1);
    byteHigher = luaL_checkint(L, i+2);
    byteHighest = luaL_checkint(L, i+3);
    // TODO: Check the sign
    dword = ((byteHighest << 24) & 0xFF000000)
          + ((byteHigher << 16) & 0x00FF0000)
          + ((byteHigh << 8) & 0x0000FF00)
          + byteLow;
    //printf("%2.2x %2.2x %2.2x %2.2x = %d\n",byteHighest, byteHigher,byteHigh,byteLow,dword);
    /* Push to stack and keep track of how much we pushed (ret) */
    // All 4 byte registers are signed numbers
    lua_pushnumber(L, (int32_t)dword);
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
  {"write_dword", lua_dynamixel_instruction_write_dword},
  {"read_data", lua_dynamixel_instruction_read_data},
  {"sync_write", lua_dynamixel_instruction_sync_write},
  {"sync_read",  lua_dynamixel_instruction_sync_read},
  {"bulk_write", lua_dynamixel_instruction_bulk_write},
	{"bulk_read", lua_dynamixel_instruction_bulk_read},
  {"word_to_byte", lua_dynamixel_word_to_byte},
  {"dword_to_byte", lua_dynamixel_dword_to_byte},
  {"byte_to_word", lua_dynamixel_byte_to_word},
  {"byte_to_dword", lua_dynamixel_byte_to_dword},
  {"crc16", lua_crc16},
  {"strerr",lua_dynamixel_error},
  {NULL, NULL}
};

/*
static const struct luaL_Reg dynamixelpacket_methods[] = {
  {NULL, NULL}
};
*/

#ifdef __cplusplus
extern "C"
#endif
int luaopen_DynamixelPacket2 (lua_State *L) {

#if LUA_VERSION_NUM == 502
  luaL_newlib( L, dynamixelpacket_functions);
#else
  luaL_register(L, "DynamixelPacket2", dynamixelpacket_functions);
#endif

  return 1;
}
