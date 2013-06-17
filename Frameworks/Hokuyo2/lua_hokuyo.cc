//Parse info
// Parse
#include <lua.hpp>
#include <stdlib.h>
#include <stdio.h>
#include <cstring>
#include <string>

#include <iostream>

using namespace std;

#include "hokuyo.h"

#define MT_NAME "hokuyo_mt"

static struct_hokuyo* lua_checkhokuyo(lua_State *L, int narg) {
  void *ud = luaL_checkudata(L, narg, MT_NAME);
  luaL_argcheck(L, ud != NULL, narg, "invalid hokuyo udata");
  return (struct_hokuyo *) ud;
}

static int lua_hokuyo_index(lua_State *L) {
  struct_hokuyo *ud = lua_checkhokuyo(L, 1);
  if (!lua_getmetatable(L, 1)) {lua_pop(L, 1); return 0;}
  lua_pushvalue(L, 2);
  lua_rawget(L, -2);
  lua_remove(L, -2);
  return 1;
}

static int lua_hokuyo_delete(lua_State *L) {
  struct_hokuyo *ud = lua_checkhokuyo(L, 1);
  int ret;
  ret = hokuyo_stream_off(ud);
  ret = hokuyo_close(ud);
  if (ret < 0) {
    return luaL_error(L, "unable to close hokuyo");
  }

  return 1;
}

static int lua_hokuyo_init(lua_State* L) { 
  /* Create new hokuyo instance */
  struct_hokuyo *ud = (struct_hokuyo *)lua_newuserdata(L, sizeof(struct_hokuyo));

  /* 1st input: Device */
  size_t device_name_len = 0;
  ud->device = lua_tolstring(L, 1, &device_name_len);

  /* 2nd input: Serial number */
  size_t serial_num_len = 0;
  ud->serial = lua_tolstring(L, 2, &serial_num_len);

  int ret = hokuyo_open(ud);
  if (ret < 0) {
    return luaL_error(L, "unable to open device");
  }

//  ret = hokuyo_stream_off(ud); 
//  printf("turn off lidar %d\n", ret);
  ret = hokuyo_stream_on(ud); 
  printf("turn on lidar %d\n", ret);

  luaL_getmetatable(L, MT_NAME);
  lua_setmetatable(L, -2);
}

static int lua_hokuyo_get(lua_State *L) {
  struct_hokuyo *ud = lua_checkhokuyo(L, 1);

  int ret = hokuyo_read(ud);
  printf("%d\n", ret);
  return 1;
}

#define HOKUYO_RAW_PACKET_LEN 3372

inline int strfind(const char * str, char c) {
  int i = 0;
  for (i = 0; i < strlen(str); i++)
    if (*(str + i) == c) 
      return i;
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

  const char * ptr = packet;
  /* Get end pos of command echo */
  int lf_pos = strfind(ptr, '\n');
  ptr += (lf_pos + 1); 

  /* Get status code */
  lf_pos = strfind(ptr, '\n');
  ptr += (lf_pos + 1);

	/* start finding section */
  lf_pos = strfind(ptr, '\n');
	char section[255];
	int sections = 0, semi_pos = 0, col_pos = 0;
	memset(section, 0, 255 * sizeof(char));
	lua_createtable(L, num, 0);
	while (lf_pos >= 0) {
		memset(section, 0, 255 * sizeof(char));
		sections ++;
		col_pos = strfind(ptr, ':');
		semi_pos = strfind(ptr, ';');
		if (col_pos >= 0 && semi_pos >= 0) {
			memcpy(section, ptr + col_pos + 1, (semi_pos - col_pos - 1) *	sizeof(char));
//			printf("%s %d %d\n", section, col_pos, semi_pos);
			lua_pushlstring(L,  section, semi_pos - col_pos -1);
			lua_rawseti(L, -2, sections);
		}
  	ptr += (lf_pos + 1);
  	lf_pos = strfind(ptr, '\n');
	}

  return 1;
}

static int lua_hokuyo_parse(lua_State *L) {
  size_t packet_len = 0;
  const char * packet = lua_tolstring(L, 1, &packet_len);

  const char * ptr = packet;
  /* Get end pos of command echo */
  int lf_pos = strfind(ptr, '\n');
  ptr += (lf_pos + 1); 

  /* Get status code */
  lf_pos = strfind(ptr, '\n');
  if (strncmp(ptr, "99", 2) != 0){
    lua_pushnil(L);
    return 1;
  }
  ptr += (lf_pos + 1);

  /* Get timestamp */
  lf_pos = strfind(ptr, '\n');
  ptr += (lf_pos + 1);

#define LINE_LEN 65
#define CLINE_LEN 64
#define MAX_COUNT 1081
#define MAX_COUNT_CHARS 3243
  char raw[MAX_COUNT_CHARS];
  char *rawp = &raw[0];
  memset(raw, 0, MAX_COUNT_CHARS * sizeof(char));
  float scan[MAX_COUNT];
  unsigned int raw_scan = 0;
  int scan_idx = 0, j = 0;
  for (int i = 0; i < 50; i++) {
    if (checksum(ptr, LINE_LEN) < 0) {
      printf("%s %d", ptr, i);
      return luaL_error(L, "checksum fail\n");
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

static const struct luaL_Reg hokuyo_Functions [] = {
  {"init", lua_hokuyo_init},
  {"parse", lua_hokuyo_parse},
  {"parse_info", lua_hokuyo_parse_info},
//  {"update", lua_hokuyo_update},
//  {"retrieve", lua_hokuyo_retrieve},
//  {"retrieve2", lua_hokuyo_retrieve2},
//  {"shutdown", lua_hokuyo_shutdown}, 
  {NULL, NULL}
};

static const struct luaL_Reg hokuyo_Methods [] = {
  {"get", lua_hokuyo_get}, 
  {"__gc", lua_hokuyo_delete},
  {"__index", lua_hokuyo_index},
  {NULL, NULL}
};

#ifdef __cplusplus
extern "C"
#endif
int luaopen_hokuyo(lua_State *L) {
  luaL_newmetatable(L, MT_NAME);

#if LUA_VERSION_NUM == 502
  luaL_setfuncs(L, hokuyo_Methods, 0);
  luaL_newlib( L, hokuyo_Functions);
#else
  luaL_register(L, NULL, hokuyo_Methods);
  luaL_register(L, "hokuyo", hokuyo_Functions);
#endif
  return 1;
}
