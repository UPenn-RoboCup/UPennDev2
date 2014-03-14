/* 
 * Enhanced MessagePack Module for Lua
 *
 * Copyright [2013] [ Yida Zhang <yida@seas.upenn.edu> ]
 *              University of Pennsylvania
 * Copyright [2014] [ Stephen McGill <smcgill3@seas.upenn.edu> ]
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

#include "lua.h"
#include "lualib.h"
#include "lauxlib.h"
#include <msgpack.h>

#ifdef TORCH
#include <torch/luaT.h>
#include <torch/TH/TH.h>
#endif

#ifdef DEBUG
#include <stdio.h>
#endif

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
	
  lua_checkunpacker(L, 1);
	
  /* Get index through metatable: */
  if (!lua_getmetatable(L, 1)) {
		lua_pop(L, 1);
		return 0;
	}/* push metatable */
	
  lua_pushvalue(L, 2); /* copy key */
  lua_rawget(L, -2); /* get metatable function */
  lua_remove(L, -2); /* delete metatable */
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
/*
	size_t sz = obj.via.raw.ptr;
  THByteTensor * tensorp = THByteTensor_newWithSize1d(sz);
  void * dest_data = tensorp->storage->data;
  memcpy(dest_data, obj.via.raw.ptr, sz);
  luaT_pushudata(L, tensorp, "torch.ByteTensor");
*/
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
		msgpack_object val = obj.via.map.ptr[i].val;
		//
    ret = (*unPackMap[key.type])(L, key);
    ret = (*unPackMap[val.type])(L, val);
		//
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
	if(value){
		msgpack_pack_true(pk);
	} else {
		msgpack_pack_false(pk);
	}
  return 1;
}

static int lua_msgpack_pack_lightuserdata(lua_State *L, int index, msgpack_packer *pk) {
	if(!lua_islightuserdata(L, index)){
		return luaL_error(L, "Input not light user data");
	}
  void *data = (void *)lua_touserdata(L, index);
  if ( data == NULL){
		return luaL_error(L, "NULL light userdata");
	}

	// TODO: Packing lightuserdata should be a bit more careful...
  int size = luaL_optint(L, index + 1, 0);

  int ret = msgpack_pack_raw(pk, size);
  ret = msgpack_pack_raw_body(pk, data, size);
  return 1;
}

static int lua_msgpack_pack_number(lua_State *L, int index, msgpack_packer *pk) {
	// TODO: Respect limits of message pack (See documentation)
  double num = lua_tonumber(L, index);
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
	// TODO: Try to speed up a lot
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

	// TODO: A bit annoying; is there a quick way in lua to check if all numeric indices?
  /* if all numeric array, use array type, otherwise use map type */
  if (!allnumeric){
    msgpack_pack_array(pk, nfield);
	} else {
    msgpack_pack_map(pk, nfield);
	}

  lua_pushnil(L);
  while (lua_next(L, index)) {
		
#ifdef DEBUG
	printf("key:val|%s:%s\n",lua_typename(L, -2),lua_typename(L, -1));
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
#ifdef DEBUG
  printf("lua function type packing not implemented, return nil\n");
#endif
  msgpack_pack_nil(pk);
  return 1;
}

static int lua_msgpack_pack_userdata(lua_State *L, int index, msgpack_packer *pk) {
#ifdef DEBUG
	printf("lua userdata type packing not implemented, return nil\n");
#endif
	
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
#else
  msgpack_pack_nil(pk);
#endif
  return 1;
}

static int lua_msgpack_pack_thread(lua_State *L, int index, msgpack_packer *pk) {
#ifdef DEBUG
  printf("lua thread type packing not implemented, return nil\n");
#endif
  msgpack_pack_nil(pk);
  return 1;
}

// TODO: Push a metatable of the buffer?
// This way, we have less copying of data possibly
// Can have the direct pointer to the data
// Just memcpy it into some message...
// The tostring method would take out the string...
static int lua_msgpack_pack(lua_State *L) {
  /* creates buffer and serializer instance. */
  msgpack_sbuffer* buffer = msgpack_sbuffer_new();
  msgpack_packer* pk = msgpack_packer_new(buffer, msgpack_sbuffer_write);

  int type = lua_type(L, 1);
  (*PackMap[type])(L, 1, pk);

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

  /* deserializes it. */
  size_t offset = 0;
  msgpack_unpacked msg;
  msgpack_unpacked_init(&msg);
  if (!msgpack_unpack_next(&msg, str, size, &offset)){
    luaL_error(L, "unpack error");
	}

  /* prints the deserialized object. */
  msgpack_object obj = msg.data;
  (*unPackMap[obj.type])(L, obj);
  lua_pushnumber(L,offset);
  return 2;
}

static int lua_msgpack_newunpacker(lua_State *L) {
  structUnpacker *ud = (structUnpacker *)lua_newuserdata(L, sizeof(structUnpacker));

  ud->str = lua_tolstring(L, 1, &ud->size);

  /* Init deserialize using msgpack_unpacker */
  ud->pac = msgpack_unpacker_new(MSGPACK_UNPACKER_INIT_BUFFER_SIZE);

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

// Unpacker tostring should be able to output the lstring
// of the raw data left to be unpacked
static int lua_msgpack_unpacker(lua_State *L) {
  structUnpacker *ud = lua_checkunpacker(L, 1);
  if( msgpack_unpacker_next(ud->pac, ud->msg) ) {
    msgpack_object obj = ud->msg->data;
		/* TODO: Push the encoded data as well as the decoded data */
    (*unPackMap[obj.type])(L, obj);
  } else {
		lua_pushnil(L);
	}
  return 1;
}

static int lua_msgpack_delete(lua_State *L) {
#ifdef DEBUG
	printf("\n\n\n\n****HERE\n\n\n");
	fflush(stdout);
#endif
	
  structUnpacker *ud = lua_checkunpacker(L, 1);
	msgpack_unpacker_free(ud->pac);
	msgpack_unpacked_destroy(ud->msg);
	// TODO: These seem like we are freeing NULL always...
  if (ud->pac){
		free(ud->pac);
	}
  if (ud->msg){
		free(ud->msg);
	}
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

int luaopen_msgpack(lua_State *L) {
	// Implement metatable for unpacker
  luaL_newmetatable(L, MT_NAME);
  lua_pushstring(L, "__index");
  lua_pushcfunction(L, lua_msgpack_index);
  lua_settable(L, -3);
#if LUA_VERSION_NUM == 502
	luaL_setfuncs(L, msgpack_Methods, 0);
#else
	luaL_register(L, NULL, msgpack_Methods);
#endif
	
	// The msgpack library
#if LUA_VERSION_NUM == 502
	luaL_newlib(L, msgpack_Functions);
#else
  luaL_register(L, "msgpack", msgpack_Functions);
#endif
	
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
