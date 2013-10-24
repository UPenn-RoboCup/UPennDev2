/* 
 * Enhanced MessagePack Module for Lua
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

#ifdef TORCH

#ifdef __cplusplus
extern "C" {
#endif

#include <torch/luaT.h>
#include <torch/TH/TH.h>

#ifdef __cplusplus
}
#endif

#endif

#ifdef DEBUG
#include <iostream>
#endif

#include <stdint.h>
#include <unistd.h>
#include <float.h>
#include <limits.h>
#include <stdio.h>
#include <math.h>
#include <msgpack.h>

#define MT_NAME "msgpack_mt"

#ifdef TORCH
/* Keep pointers to Torch objects */
static THByteTensor * b_t;
static THCharTensor * c_t;
static THShortTensor * s_t;
static THIntTensor * i_t;
static THLongTensor * l_t;
static THFloatTensor * f_t;
static THDoubleTensor * d_t;
#endif

typedef struct {
  const char *str;
  size_t size;
  msgpack_unpacker *pac;
  msgpack_unpacked *msg;
} structUnpacker;

int (*PackMap[9]) (lua_State *L, int index, msgpack_packer *pk, const char * opt);
int (*unPackMap[8]) (lua_State *L, msgpack_object obj, const char * opt);

static structUnpacker * lua_checkunpacker(lua_State *L, int narg) {
  void *ud = luaL_checkudata(L, narg, MT_NAME);
  luaL_argcheck(L, ud != NULL, narg, "invalid unpacker");
  return (structUnpacker *)ud;
}

static int lua_msgpack_index(lua_State *L) {
  lua_checkunpacker(L, 1);
  /* Get index through metatable: */
  if (!lua_getmetatable(L, 1)) {lua_pop(L, 1); return 0;} /* push metatable */
  lua_pushvalue(L, 2); /* copy key */
  lua_rawget(L, -2); /* get metatable function */
  lua_remove(L, -2); /* delete metatable */
  return 1;
}

static int lua_msgpack_unpack_nil(lua_State *L, msgpack_object obj, const char * opt) {
  lua_pushnil(L);
  return 1;
}

static int lua_msgpack_unpack_boolean(lua_State *L, msgpack_object obj, const char * opt) {
#ifdef TORCH
  if (!strcmp(opt, "torch")) {
    THCharTensor *bp = THCharTensor_newWithSize1d(1);
    THTensor_fastSet1d(bp, 0, obj.via.boolean);
    luaT_pushudata(L, bp, "torch.CharTensor");
  } else
#endif
    lua_pushboolean(L, obj.via.boolean);
  return 1;
}

static int lua_msgpack_unpack_positive_integer(lua_State *L, msgpack_object obj, const char * opt) {
#ifdef TORCH
  if (!strcmp(opt, "torch")) {
    THDoubleTensor *up = THDoubleTensor_newWithSize1d(1);
    THTensor_fastSet1d(up, 0, obj.via.u64);
    luaT_pushudata(L, up, "torch.DoubleTensor");
  } else
#endif
  lua_pushnumber(L, obj.via.u64);
  return 1;
}

static int lua_msgpack_unpack_negative_integer(lua_State *L, msgpack_object obj, const char * opt) {
#ifdef TORCH
  if (!strcmp(opt, "torch")) {
    THDoubleTensor *lp = THDoubleTensor_newWithSize1d(1);
    THTensor_fastSet1d(lp, 0, obj.via.i64);
    luaT_pushudata(L, lp, "torch.DoubleTensor");
  } else
#endif
  lua_pushnumber(L, obj.via.i64);
  return 1;
}

static int lua_msgpack_unpack_double(lua_State *L, msgpack_object obj, const char * opt) {
#ifdef TORCH
  if (!strcmp(opt, "torch")) {
    THDoubleTensor *dp = THDoubleTensor_newWithSize1d(1);
    THTensor_fastSet1d(dp, 0, obj.via.dec);
    luaT_pushudata(L, dp, "torch.DoubleTensor");
  } else
#endif
  lua_pushnumber(L, obj.via.dec);
  return 1;
}

#ifdef TORCH
template<typename T, typename TT>
static int lua_msgpack_unpack_torch(lua_State *L, TT* (*new_func)(long size), 
                             const char * Ttype, const char *ptr, uint32_t size) {
  long type_size = size / sizeof(T);
  TT * tensorp = (*new_func)(type_size);
  size_t offset = tensorp->storageOffset;
  void * dest_data = tensorp->storage->data + offset;
  memcpy(dest_data, ptr, size * sizeof(char));
  luaT_pushudata(L, tensorp, Ttype);
 
  return 1;
}
#endif

static int lua_msgpack_unpack_raw(lua_State *L, msgpack_object obj, const char * opt) {
#ifdef TORCH
  if (!strcmp(opt, "torch")) {
    const char * type = luaL_optstring(L, -1, "double");
    if (!strcmp(type, "double"))
      return lua_msgpack_unpack_torch<double, THDoubleTensor>(L, 
          THDoubleTensor_newWithSize1d, "torch.DoubleTensor", 
          obj.via.raw.ptr, obj.via.raw.size);
    else if (!strcmp(type, "float"))
      return lua_msgpack_unpack_torch<float, THFloatTensor>(L, 
          THFloatTensor_newWithSize1d, "torch.FloatTensor",
          obj.via.raw.ptr, obj.via.raw.size);
    else if (!strcmp(type, "int"))
      return lua_msgpack_unpack_torch<int, THIntTensor>(L, 
              THIntTensor_newWithSize1d, "torch.IntTensor", 
              obj.via.raw.ptr, obj.via.raw.size);
    else if (!strcmp(type, "long"))
      return lua_msgpack_unpack_torch<long, THLongTensor>(L, 
              THLongTensor_newWithSize1d, "torch.LongTensor",
              obj.via.raw.ptr, obj.via.raw.size);
    else if (!strcmp(type, "byte"))
      return lua_msgpack_unpack_torch<char, THCharTensor>(L, 
              THCharTensor_newWithSize1d, "torch.CharTensor",
              obj.via.raw.ptr, obj.via.raw.size);
    else
      luaL_error(L, "unknown type");
  } else
#endif
  lua_pushlstring(L, obj.via.raw.ptr, obj.via.raw.size);
  return 1;
}

static int lua_msgpack_unpack_array(lua_State *L, msgpack_object obj, const char * opt) {
  int i, ret;
  lua_createtable(L, obj.via.array.size, 0);
  for (i = 0; i < obj.via.array.size; i++) {
    msgpack_object ob = obj.via.array.ptr[i];
    ret = (*unPackMap[ob.type])(L, ob, opt);
    lua_rawseti(L, -2, i + 1);
  }
  return 1;
}

static int lua_msgpack_unpack_map(lua_State *L, msgpack_object obj, const char * opt) {
  int i, ret;
  lua_createtable(L, 0, obj.via.map.size);
  for (i = 0; i < obj.via.map.size; i++) {
    msgpack_object key = obj.via.map.ptr[i].key;
    ret = (*unPackMap[key.type])(L, key, opt);
    msgpack_object val = obj.via.map.ptr[i].val;
    ret = (*unPackMap[val.type])(L, val, opt);
    lua_settable(L, -3);
  }
  return 1;
}

static int lua_msgpack_pack_nil(lua_State *L, int index, msgpack_packer *pk, const char * opt) {
  msgpack_pack_nil(pk);
  return 1;
}

static int lua_msgpack_pack_boolean(lua_State *L, int index, msgpack_packer *pk, const char * opt) {
  int value = lua_toboolean(L, index);
  (value == 0)? msgpack_pack_true(pk) : msgpack_pack_false(pk);
  return 1;
}

static int lua_msgpack_pack_lightuserdata(lua_State *L, int index, msgpack_packer *pk, const char * opt) {
  uint8_t *data = (uint8_t *)lua_touserdata(L, index);
  if ((data == NULL) || !lua_islightuserdata(L, index)) 
    return luaL_error(L, "Input not light user data");

  int size = luaL_optint(L, index + 1, 0);

  int ret = msgpack_pack_raw(pk, size);
  ret = msgpack_pack_raw_body(pk, data, size);
  return 1;
}

static int lua_msgpack_pack_number(lua_State *L, int index, msgpack_packer *pk, const char * opt) {
  /* magic number : when input larger then this number
   * msgpack_pack_int64 not working as expected */
#define LIMIT 4294967296
  double num = lua_tonumber(L, index);
  double intpart;
  if (modf(num, &intpart) == 0.0) {
    (intpart > LIMIT || intpart < -LIMIT)? msgpack_pack_double(pk, intpart) : msgpack_pack_int64(pk, intpart);
  } else
    msgpack_pack_double(pk, num);
  return 1;
}

static int lua_msgpack_pack_string(lua_State *L, int index, msgpack_packer *pk, const char * opt) {
  size_t size;
  const char *str = lua_tolstring(L, index, &size);
  int ret = msgpack_pack_raw(pk, size);
  ret = msgpack_pack_raw_body(pk, str, size);
  return ret;
}

static int lua_msgpack_pack_table(lua_State *L, int index, msgpack_packer *pk, const char * opt) {
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
      ret = (*PackMap[keytype])(L, index + 1, pk, opt);
    }

    valtype = lua_type(L, -1);
    /* since value is second push to stack, the absolute position
     * on stack should be index + 2 */
    ret = (*PackMap[valtype])(L, index + 2, pk, opt);

    lua_pop(L, 1);
  }

  return 1;
}

static int lua_msgpack_pack_function(lua_State *L, int index, msgpack_packer *pk, const char * opt) {
  printf("lua function type packing not implemented, return nil\n");
  msgpack_pack_nil(pk);
  return 1;
}

static int lua_msgpack_pack_userdata(lua_State *L, int index, msgpack_packer *pk, const char * opt) {
/* printf("lua userdata type packing not implemented, return nil\n"); */
#ifdef TORCH
  const char *torch_name = luaT_typename(L, index);
  // just use byte tensor - should be fine
  THByteTensor *tensorp  = (THByteTensor *)luaT_checkudata(L, index, torch_name);
  THArgCheck(tensorp->nDimension==1, 1, "Tensor must have only one dimension.");
  
  // find how many bytes
  size_t nbytes = tensorp->size[0];
  // switch on the type
  switch(torch_name[6]){
    case 'B':
      nbytes*=sizeof(unsigned char);
      break;
    case 'C':
      nbytes*=sizeof(char);
      break;
    case 'S':
      nbytes*=sizeof(short);
      break;
    case 'L':
      nbytes*=sizeof(long);
      break;
    case 'I':
    case 'U':
      nbytes*=sizeof(int);
      break;
    case 'F':
      nbytes*=sizeof(float);
      break;
    case 'D':
      nbytes*=sizeof(double);
      break;
    default:
      return luaL_error(L, "unknown Torch Tensor type ");
  }

  // set the raw header in msgpack
  msgpack_pack_raw(pk, nbytes);
  // find the torch data
  void * src_data = tensorp->storage->data + tensorp->storageOffset;
  // pack the data as raw
  msgpack_pack_raw_body(pk, src_data, nbytes);
  return 1;
#else
  return luaL_error(L, "unknown Torch Tensor type ");
#endif
  
}

static int lua_msgpack_pack_thread(lua_State *L, int index, msgpack_packer *pk, const char * opt) {
  printf("lua thread type packing not implemented, return nil\n");
  msgpack_pack_nil(pk);
  return 1;
}

static int lua_msgpack_pack(lua_State *L) {
  /* creates buffer and serializer instance. */
  msgpack_sbuffer* buffer = msgpack_sbuffer_new();
  msgpack_packer* pk = msgpack_packer_new(buffer, msgpack_sbuffer_write);

  int type = lua_type(L, 1);
  const char * opt = luaL_optstring(L, 2, "");

  (*PackMap[type])(L, 1, pk, opt);
  //int ret = (*PackMap[type])(L, 1, pk);
  /* output packed string */
  lua_pushlstring(L, buffer->data, buffer->size);
  /* cleaning */
  msgpack_sbuffer_free(buffer);
  msgpack_packer_free(pk);
  return 1;
}

static int lua_msgpack_unpack(lua_State *L) {
  size_t size;
  const char *str  = lua_tolstring(L, 1, &size);
  const char * opt = luaL_optstring(L, 2, "");

  /* deserializes it. */
  size_t offset = 0;
  msgpack_unpacked msg;
  msgpack_unpacked_init(&msg);
  if (!msgpack_unpack_next(&msg, str, size, &offset))
    luaL_error(L, "unpack error");

  /* prints the deserialized object. */
  msgpack_object obj = msg.data;
  (*unPackMap[obj.type])(L, obj, opt);
  lua_pushnumber(L,offset);
  return 2;
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
  structUnpacker *ud = lua_checkunpacker(L, 1);
  const char *opt = luaL_optstring(L, 2, "");
  bool ret = msgpack_unpacker_next(ud->pac, ud->msg);
  if (ret) {
    msgpack_object obj = ud->msg->data;
		/* TODO: Push the encoded data as well as the decoded data */
    int re = (*unPackMap[obj.type])(L, obj, opt);
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
