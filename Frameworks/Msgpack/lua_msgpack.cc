/* 
 * MessagePack for Lua
 *
 * Copyright [2013] [ Yida Zhang <yida@seas.upenn.edu> ]
 *              University of Pennsylvania
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * */

#include <lua.hpp>

#include <stdint.h>
#include <unistd.h>
#include <float.h>
#include <limits.h>
#include <stdio.h>
#include <math.h>
#include <msgpack.h>
#include <iostream>

#define MT_NAME "msgpack_mt"

typedef struct {
  const char *str;
  size_t size;
  msgpack_unpacker *pac;
  msgpack_unpacked *msg;
} structUnpacker;

int (*PackMap[9]) (lua_State *L, int index, msgpack_packer *pk);
int (*unPackMap[8]) (lua_State *L, msgpack_object obj);

static structUnpacker * lua_checkunpacker(lua_State *L, int narg) {
  void *ud = luaL_checkudata(L, narg, MT_NAME);
  luaL_argcheck(L, ud != NULL, narg, "invalid unpacker");
  return (structUnpacker *)ud;
}

static int lua_msgpack_index(lua_State *L) {
  structUnpacker *p = lua_checkunpacker(L, 1);
  // Get index through metatable:
  if (!lua_getmetatable(L, 1)) {lua_pop(L, 1); return 0;} // push metatable
  lua_pushvalue(L, 2); // copy key
  lua_rawget(L, -2); // get metatable function
  lua_remove(L, -2); // delete metatable
  return 1;
}

static int lua_msgpack_unpack_nil(lua_State *L, msgpack_object obj) {
  lua_pushnil(L);
  return 1;
}

static int lua_msgpack_unpack_boolean(lua_State *L, msgpack_object obj) {
  lua_pushboolean(L, obj.via.boolean);
  return 1;
}

static int lua_msgpack_unpack_positive_integer(lua_State *L, msgpack_object obj) {
  lua_pushnumber(L, obj.via.u64);
  return 1;
}

static int lua_msgpack_unpack_negative_integer(lua_State *L, msgpack_object obj) {
  lua_pushnumber(L, obj.via.i64);
  return 1;
}

static int lua_msgpack_unpack_double(lua_State *L, msgpack_object obj) {
  lua_pushnumber(L, obj.via.dec);
  return 1;
}

static int lua_msgpack_unpack_raw(lua_State *L, msgpack_object obj) {
  lua_pushlstring(L, obj.via.raw.ptr, obj.via.raw.size);
  return 1;
}

static int lua_msgpack_unpack_array(lua_State *L, msgpack_object obj) {
  int i, ret;
  lua_createtable(L, obj.via.array.size, 0);
  for (i = 0; i < obj.via.array.size; i++) {
    msgpack_object ob = obj.via.array.ptr[i];
    ret = (*unPackMap[ob.type])(L, ob);
    lua_rawseti(L, -2, i + 1);
  }
  return 1;
}

static int lua_msgpack_unpack_map(lua_State *L, msgpack_object obj) {
  int i, ret;
  lua_createtable(L, 0, obj.via.map.size);
  for (i = 0; i < obj.via.map.size; i++) {
    msgpack_object key = obj.via.map.ptr[i].key;
    ret = (*unPackMap[key.type])(L, key);
    msgpack_object val = obj.via.map.ptr[i].val;
    ret = (*unPackMap[val.type])(L, val);
    lua_settable(L, -3);
  }
  return 1;
}

static int lua_msgpack_pack_nil(lua_State *L, int index, msgpack_packer *pk) {
  msgpack_pack_nil(pk);
  return 1;
}

static int lua_msgpack_pack_boolean(lua_State *L, int index, msgpack_packer *pk) {
  int value = lua_toboolean(L, index);
  (value == 0)? msgpack_pack_true(pk) : msgpack_pack_false(pk);
  return 1;
}

static int lua_msgpack_pack_lightuserdata(lua_State *L, int index, msgpack_packer *pk) {
  uint8_t *data = (uint8_t *)lua_touserdata(L, index);
  if ((data == NULL) || !lua_islightuserdata(L, index)) 
    return luaL_error(L, "Input not light user data");

  int size = luaL_optint(L, index + 1, 0);

  int ret = msgpack_pack_raw(pk, size);
  ret = msgpack_pack_raw_body(pk, data, size);
  return 1;
}

static int lua_msgpack_pack_number(lua_State *L, int index, msgpack_packer *pk) {
#define LIMIT 4294967296
  double num = lua_tonumber(L, index);
  double intpart;
  if (modf(num, &intpart) == 0.0) {
    (intpart > LIMIT || intpart < -LIMIT)? msgpack_pack_double(pk, intpart) : msgpack_pack_int64(pk, intpart);
  } else
    msgpack_pack_double(pk, num);
  return 1;
}

static int lua_msgpack_pack_string(lua_State *L, int index, msgpack_packer *pk) {
  size_t size;
  const char *str = lua_tolstring(L, index, &size);
  int ret = msgpack_pack_raw(pk, size);
  ret = msgpack_pack_raw_body(pk, str, size);
  return ret;
}

static int lua_msgpack_pack_table(lua_State *L, int index, msgpack_packer *pk) {
  int valtype, keytype, ret, allnumeric = 0;
  int nfield = 0;
  /* 
   * Use lua_next to read table
   *
   * lua_next(L, index) do three steps
   *    1. pop one key from top of stack (-1)
   *    2. get one key-value pair from stack position index and
   *       first push key into stack and then push value into stack
   *       so key at -2 and value at -1
   *    3. return 0 if step 2 succeed
   */
  /* first iterate table to get actual key-value pair numbers */
  lua_pushnil(L); /* push nil key as dummy starting point for lua_next */
  while (lua_next(L, index)) {
    nfield ++;
    if (lua_type(L, -2) != LUA_TNUMBER) allnumeric++;

    /* only pop value and leave key on stack for next lua_next operation */
    lua_pop(L, 1);
  }

  /* if all numeric array, use array type, otherwise use map type */
  if (!allnumeric) 
    msgpack_pack_array(pk, nfield);
  else
    msgpack_pack_map(pk, nfield);

  lua_pushnil(L);
  while (lua_next(L, index)) {
#ifdef DEBUG
    keytype = lua_type(L, -2);
    std::cout << "keytype " << keytype << std::endl;
 
    valtype = lua_type(L, -1);
    std::cout << "valtype " << valtype << std::endl;
#endif
    /* if all numeric array, no need to get key */
    if (allnumeric > 0) {
      keytype = lua_type(L, -2);
      /* since key is first push to stack, the absolute key position
       * on stack should be index + 1 */
      ret = (*PackMap[keytype])(L, index + 1, pk);
    }

    valtype = lua_type(L, -1);
    /* since value is second push to stack, the absolute position
     * on stack should be index + 2 */
    ret = (*PackMap[valtype])(L, index + 2, pk);

    lua_pop(L, 1);
  }

  return 1;
}

static int lua_msgpack_pack_function(lua_State *L, int index, msgpack_packer *pk) {
  msgpack_pack_nil(pk);
  return 1;
}

static int lua_msgpack_pack_userdata(lua_State *L, int index, msgpack_packer *pk) {
  msgpack_pack_nil(pk);
  return 1;
}

static int lua_msgpack_pack_thread(lua_State *L, int index, msgpack_packer *pk) {
  msgpack_pack_nil(pk);
  return 1;
}

static int lua_msgpack_pack(lua_State *L) {
  /* creates buffer and serializer instance. */
  msgpack_sbuffer* buffer = msgpack_sbuffer_new();
  msgpack_packer* pk = msgpack_packer_new(buffer, msgpack_sbuffer_write);

  int type = lua_type(L, 1);
  int ret = (*PackMap[type])(L, 1, pk);
  /* output packed string */
  lua_pushlstring(L, buffer->data, buffer->size);
  /* cleaning */
  msgpack_sbuffer_free(buffer);
  msgpack_packer_free(pk);
  return 1;
}

static int lua_msgpack_unpack(lua_State *L) {
  size_t size;
  const char *str = lua_tolstring(L, 1, &size);
  /* deserializes it. */
  msgpack_unpacked msg;
  msgpack_unpacked_init(&msg);
  if (!msgpack_unpack_next(&msg, str, size, NULL)) 
    luaL_error(L, "unpack error");

  /* prints the deserialized object. */
  msgpack_object obj = msg.data;
  int ret = (*unPackMap[obj.type])(L, obj);
  return 1;
}

static int lua_msgpack_newunpacker(lua_State *L) {
  structUnpacker *ud = (structUnpacker *)lua_newuserdata(L, sizeof(structUnpacker));

  ud->str = lua_tolstring(L, 1, &ud->size);

  /* Init deserialize using msgpack_unpacker */
  ud->pac = (msgpack_unpacker *)malloc(sizeof(msgpack_unpacker));
  msgpack_unpacker_init(ud->pac, MSGPACK_UNPACKER_INIT_BUFFER_SIZE);

  /* feeds the buffer */
  msgpack_unpacker_reserve_buffer(ud->pac, ud->size);
  memcpy(msgpack_unpacker_buffer(ud->pac), ud->str, ud->size);
  msgpack_unpacker_buffer_consumed(ud->pac, ud->size);

  /* start streaming deserialization */
  ud->msg = (msgpack_unpacked *)malloc(sizeof(msgpack_unpacked));
  msgpack_unpacked_init(ud->msg);

  luaL_getmetatable(L, MT_NAME);
  lua_setmetatable(L, -2);
  return 1;
}

static int lua_msgpack_unpacker(lua_State *L) {
  int re = 0;
  structUnpacker *ud = lua_checkunpacker(L, 1);
  bool ret = msgpack_unpacker_next(ud->pac, ud->msg);
  if (ret) {
    msgpack_object obj = ud->msg->data;
    re = (*unPackMap[obj.type])(L, obj);
  } else lua_pushnil(L);
  return 1;
}

static int lua_msgpack_delete(lua_State *L) {
  structUnpacker *ud = lua_checkunpacker(L, 1);
  if (!ud->pac) free(ud->pac);
  if (!ud->msg) free(ud->msg);

  return 1;
}

static const struct luaL_reg msgpack_Functions [] = {
  {"pack", lua_msgpack_pack},
  {"unpack", lua_msgpack_unpack},
  {"unpacker", lua_msgpack_newunpacker},
  {NULL, NULL}
};

static const struct luaL_reg msgpack_Methods [] = {
  {"unpack", lua_msgpack_unpacker},
  {"__gc", lua_msgpack_delete},
  {NULL, NULL}
};

#ifdef __cplusplus
extern "C"
#endif
int luaopen_msgpack(lua_State *L) {
  luaL_newmetatable(L, MT_NAME);

  // Implement index method:
  lua_pushstring(L, "__index");
  lua_pushcfunction(L, lua_msgpack_index);
  lua_settable(L, -3);

  luaL_register(L, NULL, msgpack_Methods);
  luaL_register(L, "msgpack", msgpack_Functions);

  /* Init pack functions map */
  PackMap[LUA_TNIL] = lua_msgpack_pack_nil;
  PackMap[LUA_TBOOLEAN] = lua_msgpack_pack_boolean;
  PackMap[LUA_TLIGHTUSERDATA] = lua_msgpack_pack_lightuserdata;
  PackMap[LUA_TNUMBER] = lua_msgpack_pack_number;
  PackMap[LUA_TSTRING] = lua_msgpack_pack_string;
  PackMap[LUA_TTABLE] = lua_msgpack_pack_table;
  PackMap[LUA_TFUNCTION] = lua_msgpack_pack_function;
  PackMap[LUA_TUSERDATA] = lua_msgpack_pack_userdata;
  PackMap[LUA_TTHREAD] = lua_msgpack_pack_thread;
 
  /* Init unpack functions Map */
  unPackMap[MSGPACK_OBJECT_NIL] = lua_msgpack_unpack_nil;
  unPackMap[MSGPACK_OBJECT_BOOLEAN] = lua_msgpack_unpack_boolean;
  unPackMap[MSGPACK_OBJECT_POSITIVE_INTEGER] = lua_msgpack_unpack_positive_integer;
  unPackMap[MSGPACK_OBJECT_NEGATIVE_INTEGER] = lua_msgpack_unpack_negative_integer;
  unPackMap[MSGPACK_OBJECT_DOUBLE] = lua_msgpack_unpack_double;
  unPackMap[MSGPACK_OBJECT_RAW] = lua_msgpack_unpack_raw;
  unPackMap[MSGPACK_OBJECT_ARRAY] = lua_msgpack_unpack_array;
  unPackMap[MSGPACK_OBJECT_MAP] = lua_msgpack_unpack_map; 

  return 1;
}
