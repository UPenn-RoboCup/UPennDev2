/*
(c) 2013 Yida Zhang, Stephen McGill
Parse Hokuyo packets
*/
#include "lua.h"
#include "lualib.h"
#include "lauxlib.h"
#include <string.h>

#define HOKUYO_RAW_PACKET_LEN 3372

inline int strfind(const char * str, char c, int length) {
	int i = 0;
	for (i = 0; i < length; i++){
		if (*(str + i) == c) {
			return i;
		}
	}
	return -1;
}

inline int checksum(const char *str, int len) {
	int i = 0;
	char sum = 0;
	for (i = 0; i < len - 1; i++) sum += str[i];
	sum = (sum & 0x3f) + 0x30;
	if (sum != str[len - 1]){
		printf("Expected (%d) got (%d)\n", sum, str[len - 1]);
		return -1;
	}
	else
		return 0;
}

static int lua_hokuyo_parse_info(lua_State *L) {
	size_t packet_len = 0;
	const char * packet = lua_tolstring(L, 1, &packet_len);
	int num = luaL_checkinteger(L, 2);
	int lf_pos;

	const char * ptr = packet;
	/* Get end pos of command echo */
	/*
	lf_pos = strfind(ptr, '\n', packet_len);
	ptr += (lf_pos + 1);
	packet_len -= (lf_pos + 1);
*/
	/* Get status code */
	lf_pos = strfind(ptr, '\n', packet_len);
	ptr += (lf_pos + 1);
	packet_len -= (lf_pos + 1);

	/* start finding section */
	lf_pos = strfind(ptr, '\n', packet_len);
	char section[255];
	int sections = 0, semi_pos = 0, col_pos = 0;
	lua_createtable(L, num, 0);
	while (lf_pos >= 0) {
		sections ++;
		col_pos  = strfind(ptr, ':', packet_len );
		semi_pos = strfind(ptr, ';', packet_len );
		if (col_pos >= 0 && semi_pos >= 0) {
			memcpy(section, ptr + col_pos + 1, (semi_pos - col_pos - 1) *	sizeof(char));
			lua_pushlstring(L,  section, semi_pos - col_pos - 1);
			lua_rawseti(L, -2, sections);
		}
		ptr += (lf_pos + 1);
		packet_len -= (lf_pos + 1);
		lf_pos = strfind(ptr, '\n', packet_len);
	}

	return 1;
}

static int lua_hokuyo_parse(lua_State *L) {
	size_t packet_len = 0;
	const char * packet = lua_tolstring(L, 1, &packet_len);
	const char * ptr = packet;
	int lf_pos;
	/* Get end pos of command echo */
	lf_pos = strfind(ptr, '\n', packet_len);
	ptr += (lf_pos + 1); 
	packet_len -= (lf_pos + 1);

	/* Get status code */
	lf_pos = strfind(ptr, '\n', packet_len);
	if ( strncmp(ptr, "99", 2) != 0){
		lua_pushnil(L);
		return 1;
	}
	ptr += (lf_pos + 1);
	packet_len -= (lf_pos + 1);

	/* Get timestamp */
	lf_pos = strfind(ptr, '\n', packet_len);
	ptr += (lf_pos + 1);
	packet_len -= (lf_pos + 1);

#define LINE_LEN 65
#define CLINE_LEN 64
#define MAX_COUNT 1081
#define MAX_COUNT_CHARS 3243
	char raw[MAX_COUNT_CHARS];
	char *rawp = &raw[0];
	float scan[MAX_COUNT];
	unsigned int raw_scan = 0;
	int scan_idx = 0, j = 0, i = 0;
	for (i = 0; i < 50; i++) {
		if (checksum(ptr, LINE_LEN) < 0) {
			printf("%s %d", ptr, i);
      lua_pushnil(L);
      return 1;
//			return luaL_error(L, "Checksum fail\n");
		}
		memcpy(rawp, ptr, CLINE_LEN * sizeof(char));
		rawp += CLINE_LEN;
		ptr = ptr + (LINE_LEN + 1);
	}
	memcpy(rawp, ptr, 42 * sizeof(char));

	for (j = 0; j < MAX_COUNT_CHARS; j+=3) {
		raw_scan = ((raw[j] - 0x30) << 12) + ((raw[j + 1] - 0x30) << 6) + raw[j + 2] - 0x30;
		scan[scan_idx++] = (float) (raw_scan * 0.001);
	}

	lua_pushlstring(L, (char *)scan, MAX_COUNT * sizeof(float));

	return 1;
}

static const struct luaL_Reg HokuyoPacket_Functions [] = {
	{"parse", lua_hokuyo_parse},
	{"parse_info", lua_hokuyo_parse_info},
	{NULL, NULL}
};

int luaopen_HokuyoPacket(lua_State *L) {

#if LUA_VERSION_NUM == 502
	luaL_newlib( L, HokuyoPacket_Functions);
#else
	luaL_register(L, "hokuyo", HokuyoPacket_Functions);
#endif
	return 1;
}
