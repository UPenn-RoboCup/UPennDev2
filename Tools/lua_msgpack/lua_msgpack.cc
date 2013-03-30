/* 
 * MessagePack for Matlab
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

#include <unistd.h>
#include <stdio.h>
#include <msgpack.h>

#define MT_NAME "msgpack_mt"

//bool debug = false;
//
//mxArray* mex_unpack_boolean(msgpack_object obj);
//mxArray* mex_unpack_positive_integer(msgpack_object obj);
//mxArray* mex_unpack_negative_integer(msgpack_object obj);
//mxArray* mex_unpack_double(msgpack_object obj);
//mxArray* mex_unpack_raw(msgpack_object obj);
//mxArray* mex_unpack_nil(msgpack_object obj);
//mxArray* mex_unpack_map(msgpack_object obj);
//mxArray* mex_unpack_array(msgpack_object obj);
//
//void (*PackMap[17]) (msgpack_packer *pk, int nrhs, const mxArray *prhs);
//mxArray* (*unPackMap[8]) (msgpack_object obj);
//
//int i = 0;
//
//mxArray* mex_unpack_boolean(msgpack_object obj) {
//  return mxCreateLogicalScalar(obj.via.boolean);
//}
//
//mxArray* mex_unpack_positive_integer(msgpack_object obj) {
//  mxArray *ret = mxCreateNumericMatrix(1,1, mxUINT64_CLASS, mxREAL);
//  uint64_t *ptr = (uint64_t *)mxGetPr(ret);
//  *ptr = obj.via.u64;
//  return ret;
//}
//
//mxArray* mex_unpack_negative_integer(msgpack_object obj) {
//  mxArray *ret = mxCreateNumericMatrix(1,1, mxINT64_CLASS, mxREAL);
//  int64_t *ptr = (int64_t *)mxGetPr(ret);
//  *ptr = obj.via.i64;
//  return ret;
//}
//
//mxArray* mex_unpack_double(msgpack_object obj) {
//  mxArray* ret = mxCreateDoubleMatrix(1,1, mxREAL);
//  double *ptr = (double *)mxGetPr(ret);
//  *ptr = obj.via.dec;
//  return ret;
//}
//
//mxArray* mex_unpack_raw(msgpack_object obj) {
//  mwSize dims[] = {obj.via.raw.size};
//  mxArray* ret = mxCreateCharArray(1, dims);
//  uint16_t *ptr = (uint16_t*)mxGetPr(ret); 
//  for (int i = 0; i < obj.via.raw.size; i++) {
//    ptr[i] = obj.via.raw.ptr[i];
//  }
//  return ret;
//}
//
//mxArray* mex_unpack_nil(msgpack_object obj) {
//  return mxCreateCellArray(0,0);
//}
//
//mxArray* mex_unpack_map(msgpack_object obj) {
//  uint32_t nfields = obj.via.map.size;
//  const char *field_name[nfields];
//  for (i = 0; i < nfields; i++) {
//    struct msgpack_object_kv obj_kv = obj.via.map.ptr[i];
//    if (obj_kv.key.type == MSGPACK_OBJECT_RAW) {
//      field_name[i] = (const char*)mxCalloc(obj_kv.key.via.raw.size, sizeof(uint8_t));
//      memcpy((char*)field_name[i], obj_kv.key.via.raw.ptr, obj_kv.key.via.raw.size);
//    }
//  }
//  mxArray *ret = mxCreateStructMatrix(1, 1, obj.via.map.size, field_name);
//  for (i = 0; i < nfields; i++) {
//    int ifield = mxGetFieldNumber(ret, field_name[i]);
//    msgpack_object ob = obj.via.map.ptr[i].val;
//    mxSetFieldByNumber(ret, 0, ifield, (*unPackMap[ob.type])(ob));
//  }
//  for (i = 0; i < nfields; i++)
//    mxFree((void *)field_name[i]);
//  return ret;
//}
//
//mxArray* mex_unpack_array(msgpack_object obj) {
//  // validata array element type
//  std::set<int> types;
//  for (i = 0; i < obj.via.array.size; i++)
//    if ((types.find(obj.via.array.ptr[i].type) == types.end()) && 
//        (obj.via.array.ptr[i].type > 0) and (obj.via.array.ptr[i].type < 5))
//      types.insert(obj.via.array.ptr[i].type);
//  int unique_type = *types.begin();
//  if (types.size() == 1) {
//    mxArray *ret = NULL;
//    bool * ptrb = NULL;
//    double * ptrd = NULL;
//    int64_t * ptri = NULL;
//    uint64_t * ptru = NULL;
//    switch (unique_type) {
//      case 1:
//        ret = mxCreateLogicalMatrix(obj.via.array.size, 1);
//        ptrb = (bool*)mxGetPr(ret);
//        for (i = 0; i < obj.via.array.size; i++) ptrb[i] = obj.via.array.ptr[i].via.boolean;
//        break;
//      case 2:
//        ret = mxCreateNumericMatrix(obj.via.array.size, 1, mxUINT64_CLASS, mxREAL);
//        ptru = (uint64_t*)mxGetPr(ret);
//        for (i = 0; i < obj.via.array.size; i++) ptru[i] = obj.via.array.ptr[i].via.u64;
//        break;
//      case 3:
//        ret = mxCreateNumericMatrix(obj.via.array.size, 1, mxINT64_CLASS, mxREAL);
//        ptri = (int64_t*)mxGetPr(ret);
//        for (i = 0; i < obj.via.array.size; i++) ptri[i] = obj.via.array.ptr[i].via.i64;
//        break;
//      case 4:
//        ret = mxCreateNumericMatrix(obj.via.array.size, 1, mxDOUBLE_CLASS, mxREAL);
//        ptrd = mxGetPr(ret);
//        for (i = 0; i < obj.via.array.size; i++) ptrd[i] = obj.via.array.ptr[i].via.dec;
//        break;
//      default:
//        break;
//    }
//    return ret;
//  }
//  else {
//    mxArray *ret = mxCreateCellMatrix(obj.via.array.size, 1);
//    for (i = 0; i < obj.via.array.size; i++) {
//      msgpack_object ob = obj.via.array.ptr[i];
//      mxSetCell(ret, i, (*unPackMap[ob.type])(ob));
//    }
//    return ret;
//  }
//}
//
//void mex_unpack(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) 
//{
//  const char *str = (const char*)mxGetPr(prhs[0]);
//  int size = mxGetM(prhs[0]) * mxGetN(prhs[0]);
//
//  /* deserializes it. */
//  msgpack_unpacked msg;
//  msgpack_unpacked_init(&msg);
//  if (!msgpack_unpack_next(&msg, str, size, NULL)) 
//    mexErrMsgTxt("unpack error");
//
//  /* prints the deserialized object. */
//  msgpack_object obj = msg.data;
//  plhs[0] = (*unPackMap[obj.type])(obj);
//}
//
//void mex_pack_single(msgpack_packer *pk, int nrhs, const mxArray *prhs) {
//  int nElements = mxGetNumberOfElements(prhs);
//  float *data = (float*)mxGetPr(prhs);
//  if (nElements > 1) msgpack_pack_array(pk, nElements);
//  for (i = 0; i < nElements; i++) {
//    msgpack_pack_float(pk, data[i]);
//  }
//}
//
//void mex_pack_double(msgpack_packer *pk, int nrhs, const mxArray *prhs) {
//  int nElements = mxGetNumberOfElements(prhs);
//  double *data = mxGetPr(prhs);
//  if (nElements > 1) msgpack_pack_array(pk, nElements);
//  for (i = 0; i < nElements; i++) {
//    msgpack_pack_double(pk, data[i]);
//  }
//}
//
//void mex_pack_int8(msgpack_packer *pk, int nrhs, const mxArray *prhs) {
//  int nElements = mxGetNumberOfElements(prhs);
//  int8_t *data = (int8_t*)mxGetPr(prhs);
//  if (nElements > 1) msgpack_pack_array(pk, nElements);
//  for (i = 0; i < nElements; i++) {
//    msgpack_pack_int8(pk, data[i]);
//  }
//}
//
//void mex_pack_uint8(msgpack_packer *pk, int nrhs, const mxArray *prhs) {
//  int nElements = mxGetNumberOfElements(prhs);
//  uint8_t *data = (uint8_t*)mxGetPr(prhs);
//  if (nElements > 1) msgpack_pack_array(pk, nElements);
//  for (i = 0; i < nElements; i++) {
//    msgpack_pack_uint8(pk, data[i]);
//  }
//}
//
//void mex_pack_int16(msgpack_packer *pk, int nrhs, const mxArray *prhs) {
//  int nElements = mxGetNumberOfElements(prhs);
//  int16_t *data = (int16_t*)mxGetPr(prhs);
//  if (nElements > 1) msgpack_pack_array(pk, nElements);
//  for (i = 0; i < nElements; i++) {
//    msgpack_pack_int16(pk, data[i]);
//  }
//}
//
//void mex_pack_uint16(msgpack_packer *pk, int nrhs, const mxArray *prhs) {
//  int nElements = mxGetNumberOfElements(prhs);
//  uint16_t *data = (uint16_t*)mxGetPr(prhs);
//  if (nElements > 1) msgpack_pack_array(pk, nElements);
//  for (i = 0; i < nElements; i++) {
//    msgpack_pack_uint16(pk, data[i]);
//  }
//}
//
//void mex_pack_int32(msgpack_packer *pk, int nrhs, const mxArray *prhs) {
//  int nElements = mxGetNumberOfElements(prhs);
//  int32_t *data = (int32_t*)mxGetPr(prhs);
//  if (nElements > 1) msgpack_pack_array(pk, nElements);
//  for (i = 0; i < nElements; i++) {
//    msgpack_pack_int32(pk, data[i]);
//  }
//}
//
//void mex_pack_uint32(msgpack_packer *pk, int nrhs, const mxArray *prhs) {
//  int nElements = mxGetNumberOfElements(prhs);
//  uint32_t *data = (uint32_t*)mxGetPr(prhs);
//  if (nElements > 1) msgpack_pack_array(pk, nElements);
//  for (i = 0; i < nElements; i++) {
//    msgpack_pack_uint32(pk, data[i]);
//  }
//}
//
//void mex_pack_int64(msgpack_packer *pk, int nrhs, const mxArray *prhs) {
//  int nElements = mxGetNumberOfElements(prhs);
//  int64_t *data = (int64_t*)mxGetPr(prhs);
//  if (nElements > 1) msgpack_pack_array(pk, nElements);
//  for (i = 0; i < nElements; i++) {
//    msgpack_pack_int64(pk, data[i]);
//  }
//}
//
//void mex_pack_uint64(msgpack_packer *pk, int nrhs, const mxArray *prhs) {
//  int nElements = mxGetNumberOfElements(prhs);
//  uint64_t *data = (uint64_t*)mxGetPr(prhs);
//  if (nElements > 1) msgpack_pack_array(pk, nElements);
//  for (i = 0; i < nElements; i++) {
//    msgpack_pack_uint64(pk, data[i]);
//  }
//}
//
//void mex_pack_logical(msgpack_packer *pk, int nrhs, const mxArray *prhs) {
//  int nElements = mxGetNumberOfElements(prhs);
//  bool *data = mxGetLogicals(prhs);
//  if (nElements > 1) msgpack_pack_array(pk, nElements);
//  for (i = 0; i < nElements; i++)
//    (data[i])? msgpack_pack_true(pk) : msgpack_pack_false(pk);
//}
//
//void mex_pack_char(msgpack_packer *pk, int nrhs, const mxArray *prhs) {
//  int nElements = mxGetNumberOfElements(prhs);
//  uint8_t *data = (uint8_t*)mxGetPr(prhs); 
//  // matlab char type is actually uint16 -> 2 * uint8
//  msgpack_pack_raw(pk, nElements * 2);
//  msgpack_pack_raw_body(pk, data, nElements * 2);
//}
//
//void mex_pack_cell(msgpack_packer *pk, int nrhs, const mxArray *prhs) {
//  int nElements = mxGetNumberOfElements(prhs);
//  if (nElements > 1) msgpack_pack_array(pk, nElements);
//  for (i = 0; i < nElements; i++) {
//    mxArray * pm = mxGetCell(prhs, i);
//    (*PackMap[mxGetClassID(pm)])(pk, nrhs, pm);
//  }
//}
//
//void mex_pack_struct(msgpack_packer *pk, int nrhs, const mxArray *prhs) {
//  int nField = mxGetNumberOfFields(prhs);
//  if (nField > 1) msgpack_pack_map(pk, nField);
//  const char* fname = NULL;
//  int fnameLen = 0;
//  int ifield = 0;
//  for (i = 0; i < nField; i++) {
//    fname = mxGetFieldNameByNumber(prhs, i);
//    fnameLen = strlen(fname);
//    msgpack_pack_raw(pk, fnameLen);
//    msgpack_pack_raw_body(pk, fname, fnameLen);
//    ifield = mxGetFieldNumber(prhs, fname);
//    mxArray* pm = mxGetFieldByNumber(prhs, 0, ifield);
//    (*PackMap[mxGetClassID(pm)])(pk, nrhs, pm);
//  }
//}
//
//void mex_pack(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
//  /* creates buffer and serializer instance. */
//  msgpack_sbuffer* buffer = msgpack_sbuffer_new();
//  msgpack_packer* pk = msgpack_packer_new(buffer, msgpack_sbuffer_write);
//
//  for (i = 0; i < nrhs; i ++)
//    (*PackMap[mxGetClassID(prhs[i])])(pk, nrhs, prhs[i]);
//
//  plhs[0] = mxCreateNumericMatrix(1, buffer->size, mxUINT8_CLASS, mxREAL);
//  memcpy(mxGetPr(plhs[0]), buffer->data, buffer->size * sizeof(uint8_t));
//
//  /* cleaning */
//  msgpack_sbuffer_free(buffer);
//  msgpack_packer_free(pk);
//}
//
//void mex_unpacker(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
//  /* Init deserialize using msgpack_unpacker */
//  msgpack_unpacker pac;
//  msgpack_unpacker_init(&pac, MSGPACK_UNPACKER_INIT_BUFFER_SIZE);
//
//  const char *str = (const char*)mxGetPr(prhs[0]);
//  int size = mxGetM(prhs[0]) * mxGetN(prhs[0]);
//  if (size) {
//    /* feeds the buffer */
//    msgpack_unpacker_reserve_buffer(&pac, size);
//    memcpy(msgpack_unpacker_buffer(&pac), str, size);
//    msgpack_unpacker_buffer_consumed(&pac, size);
//  
//    /* start streaming deserialization */
//    std::vector<mxArray*> ret;
//    msgpack_unpacked msg;
//    msgpack_unpacked_init(&msg);
//    while (msgpack_unpacker_next(&pac, &msg)) {
//      /* prints the deserialized object. */
//      msgpack_object obj = msg.data;
//      ret.push_back((*unPackMap[obj.type])(obj));
//    }
//    /* set cell for output */
//    plhs[0] = mxCreateCellMatrix(ret.size(), 1);
//    for (i = 0; i < ret.size(); i++)
//      mxSetCell((mxArray*)plhs[0], i, ret[i]);
//  }
//}
//
//void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
//{
//  /* Init unpack functions Map */
//  unPackMap[MSGPACK_OBJECT_NIL] = mex_unpack_nil;
//  unPackMap[MSGPACK_OBJECT_BOOLEAN] = mex_unpack_boolean;
//  unPackMap[MSGPACK_OBJECT_POSITIVE_INTEGER] = mex_unpack_positive_integer;
//  unPackMap[MSGPACK_OBJECT_NEGATIVE_INTEGER] = mex_unpack_negative_integer;
//  unPackMap[MSGPACK_OBJECT_DOUBLE] = mex_unpack_double;
//  unPackMap[MSGPACK_OBJECT_RAW] = mex_unpack_raw;
//  unPackMap[MSGPACK_OBJECT_ARRAY] = mex_unpack_array;
//  unPackMap[MSGPACK_OBJECT_MAP] = mex_unpack_map; 
//
//  packmap[mxunknown_class] = null;
//  packmap[mxvoid_class] = null;
//  packmap[mxfunction_class] = null;
//  packmap[mxcell_class] = mex_pack_cell;
//  packmap[mxstruct_class] = mex_pack_struct;
//  packmap[mxlogical_class] = mex_pack_logical;
//  packmap[mxchar_class] = mex_pack_char;
//  packmap[mxdouble_class] = mex_pack_double;
//  packmap[mxsingle_class] = mex_pack_single;
//  packmap[mxint8_class] = mex_pack_int8;
//  packmap[mxuint8_class] = mex_pack_uint8;
//  packmap[mxint16_class] = mex_pack_int16;
//  packmap[mxuint16_class] = mex_pack_uint16;
//  packmap[mxint32_class] = mex_pack_int32;
//  packmap[mxuint32_class] = mex_pack_uint32;
//  packmap[mxint64_class] = mex_pack_int64;
//  packmap[mxuint64_class] = mex_pack_uint64;
//
//  if ((nrhs < 1) || (!mxIsChar(prhs[0])))
//    mexErrMsgTxt("Need to input string argument");
//  char *fname = mxArrayToString(prhs[0]);
//  if (strcmp(fname, "pack") == 0)
//    mex_pack(nlhs, plhs, nrhs-1, prhs+1);
//  else if (strcmp(fname, "unpack") == 0)
//    mex_unpack(nlhs, plhs, nrhs-1, prhs+1);
//  else if (strcmp(fname, "unpacker") == 0)
//    mex_unpacker(nlhs, plhs, nrhs-1, prhs+1);
//  else
//    mexErrMsgTxt("Unknown function argument");
//}

int (*PackMap[9]) (lua_State *L, msgpack_packer *pk);

static int lua_msgpack_pack_nil(lua_State *L, msgpack_packer *pk) {
  printf("nil input\n");
  return 1;
}

static int lua_msgpack_pack_boolean(lua_State *L, msgpack_packer *pk) {
  printf("boolean input\n");
  return 1;
}

static int lua_msgpack_pack_lightuserdata(lua_State *L, msgpack_packer *pk) {
  printf("lightuserdata input\n");
  return 1;
}

static int lua_msgpack_pack_number(lua_State *L, msgpack_packer *pk) {
  printf("number input\n");
  return 1;
}

static int lua_msgpack_pack_string(lua_State *L, msgpack_packer *pk) {
  printf("string input\n");
  return 1;
}

static int lua_msgpack_pack_table(lua_State *L, msgpack_packer *pk) {
  printf("table input\n");
  return 1;
}

static int lua_msgpack_pack_function(lua_State *L, msgpack_packer *pk) {
  printf("function input\n");
  return 1;
}

static int lua_msgpack_pack_userdata(lua_State *L, msgpack_packer *pk) {
  printf("userdata input\n");
  return 1;
}

static int lua_msgpack_pack_thread(lua_State *L, msgpack_packer *pk) {
  printf("thread input\n");
  return 1;
}

static int lua_msgpack_pack(lua_State *L) {
  /* creates buffer and serializer instance. */
  msgpack_sbuffer* buffer = msgpack_sbuffer_new();
  msgpack_packer* pk = msgpack_packer_new(buffer, msgpack_sbuffer_write);

  int i, type, ret;
  int top = lua_gettop(L);
  for (i = 1; i <= top; i++) {
    type = lua_type(L, i);
    ret = (*PackMap[type])(L, pk);
  }
//  plhs[0] = mxCreateNumericMatrix(1, buffer->size, mxUINT8_CLASS, mxREAL);
//  memcpy(mxGetPr(plhs[0]), buffer->data, buffer->size * sizeof(uint8_t));
//
  /* cleaning */
  msgpack_sbuffer_free(buffer);
  msgpack_packer_free(pk);
  return 1;
}

static int lua_msgpack_unpack(lua_State *L) {
  return 1;
}

static const struct luaL_reg msgpack_Functions [] = {
  {"pack", lua_msgpack_pack},
  {"unpack", lua_msgpack_unpack},
  {NULL, NULL}
};

static const struct luaL_reg msgpack_Methods [] = {
  {NULL, NULL}
};

#ifdef __cplusplus
extern "C"
#endif
int luaopen_msgpack(lua_State *L) {
  luaL_newmetatable(L, MT_NAME);

  luaL_register(L, NULL, msgpack_Methods);
  luaL_register(L, "msgpack", msgpack_Functions);

  luaL_register(L, NULL, msgpack_Methods);
  luaL_register(L, "msgpack", msgpack_Functions);

  PackMap[LUA_TNIL] = lua_msgpack_pack_nil;
  PackMap[LUA_TBOOLEAN] = lua_msgpack_pack_boolean;
  PackMap[LUA_TLIGHTUSERDATA] = lua_msgpack_pack_lightuserdata;
  PackMap[LUA_TNUMBER] = lua_msgpack_pack_number;
  PackMap[LUA_TSTRING] = lua_msgpack_pack_string;
  PackMap[LUA_TTABLE] = lua_msgpack_pack_table;
  PackMap[LUA_TFUNCTION] = lua_msgpack_pack_function;
  PackMap[LUA_TUSERDATA] = lua_msgpack_pack_userdata;
  PackMap[LUA_TTHREAD] = lua_msgpack_pack_thread;
 

  return 1;
}
