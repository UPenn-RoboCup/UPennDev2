/*
Lua module to provide process dynamixel packets
*/

#include "dynamixel.h"
#include <lua.hpp>

#define DXL_LOBYTE(w) ((unsigned char)(((unsigned long)(w)) & 0xff))
#define DXL_HIBYTE(w) ((unsigned char)((((unsigned long)(w)) >> 8) & 0xff))
static unsigned short crc_table[256] = {0x0000,
	0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
	0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027,
	0x0022, 0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D,
	0x8077, 0x0072, 0x0050, 0x8055, 0x805F, 0x005A, 0x804B,
	0x004E, 0x0044, 0x8041, 0x80C3, 0x00C6, 0x00CC, 0x80C9,
	0x00D8, 0x80DD, 0x80D7, 0x00D2, 0x00F0, 0x80F5, 0x80FF,
	0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1, 0x00A0, 0x80A5,
	0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1, 0x8093,
	0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
	0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197,
	0x0192, 0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE,
	0x01A4, 0x81A1, 0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB,
	0x01FE, 0x01F4, 0x81F1, 0x81D3, 0x01D6, 0x01DC, 0x81D9,
	0x01C8, 0x81CD, 0x81C7, 0x01C2, 0x0140, 0x8145, 0x814F,
	0x014A, 0x815B, 0x015E, 0x0154, 0x8151, 0x8173, 0x0176,
	0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162, 0x8123,
	0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
	0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104,
	0x8101, 0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D,
	0x8317, 0x0312, 0x0330, 0x8335, 0x833F, 0x033A, 0x832B,
	0x032E, 0x0324, 0x8321, 0x0360, 0x8365, 0x836F, 0x036A,
	0x837B, 0x037E, 0x0374, 0x8371, 0x8353, 0x0356, 0x035C,
	0x8359, 0x0348, 0x834D, 0x8347, 0x0342, 0x03C0, 0x83C5,
	0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1, 0x83F3,
	0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
	0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7,
	0x03B2, 0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E,
	0x0384, 0x8381, 0x0280, 0x8285, 0x828F, 0x028A, 0x829B,
	0x029E, 0x0294, 0x8291, 0x82B3, 0x02B6, 0x02BC, 0x82B9,
	0x02A8, 0x82AD, 0x82A7, 0x02A2, 0x82E3, 0x02E6, 0x02EC,
	0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2, 0x02D0, 0x82D5,
	0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1, 0x8243,
	0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
	0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264,
	0x8261, 0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E,
	0x0234, 0x8231, 0x8213, 0x0216, 0x021C, 0x8219, 0x0208,
	0x820D, 0x8207, 0x0202 };

static unsigned short update_crc( unsigned short crc_accum, const unsigned char *data_blk_ptr, unsigned short data_blk_size ){
//	register unsigned short i, j;
	static unsigned short i, j;
	for(j=0; j<data_blk_size; j++) {
		i = ((unsigned short)(crc_accum >> 8) ^ *data_blk_ptr++) & 0xff;
		crc_accum = (crc_accum << 8) ^ crc_table[i];
	}
	return crc_accum;
}

static int lua_crc16(lua_State *L) {
	size_t nstr;
	const unsigned char *str = (unsigned char *)luaL_checklstring(L, 1, &nstr);
	unsigned short crc = update_crc(0, str, nstr);
	lua_pushnumber( L, DXL_LOBYTE(crc) );
	lua_pushnumber( L, DXL_HIBYTE(crc) );
	return 2;
}

static int lua_pushpacket(lua_State *L, DynamixelPacket *p) {
	if (p != NULL) {
		int nlen = p->length + 4;
		lua_pushlstring(L, (char *)p, nlen);
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
	int ret = 0;
	if (str) {
		for (int i = 0; i < nstr; i++) {
			nPacket = dynamixel_input(&pkt, str[i], nPacket);
			if (nPacket < 0) {
				ret += lua_pushpacket(L, &pkt);
			}
		}
	}
	return ret;
}

static int lua_dynamixel_byte_to_word(lua_State *L) {
	int n = lua_gettop(L);
	int ret = 0;
	for (int i = 1; i < n; i += 2) {
		unsigned short byteLow = luaL_checkint(L, i);
		unsigned short byteHigh = luaL_checkint(L, i+1);
		unsigned short word = (byteHigh << 8) + byteLow;
		lua_pushnumber(L, word);
		ret++;
	}
	return ret;
}

static int lua_dynamixel_word_to_byte(lua_State *L) {
	int n = lua_gettop(L);
	int ret = 0;
	for (int i = 1; i <= n; i++) {
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

static const struct luaL_reg dynamixelpacket_functions[] = {
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
	{"crc16", lua_crc16},
	{NULL, NULL}
};

static const struct luaL_reg dynamixelpacket_methods[] = {
	{NULL, NULL}
};

#ifdef __cplusplus
extern "C"
#endif
int luaopen_DynamixelPacket (lua_State *L) {
	luaL_newmetatable(L, "dynamixelpacket_mt");

	// OO access: mt.__index = mt
	// Not compatible with array access
	lua_pushvalue(L, -1);
	lua_setfield(L, -2, "__index");

	luaL_register(L, NULL, dynamixelpacket_methods);
	luaL_register(L, "DynamixelPacket", dynamixelpacket_functions);

	return 1;
}