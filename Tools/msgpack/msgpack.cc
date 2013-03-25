#include <iostream>
#include <string>
#include <map>
#include <vector>
#include <stdio.h>
#include <unistd.h>
#include <msgpack.h>

#include "mex.h"

using namespace std;

bool debug = true;
std::string msgpack_type[8];

mxArray* mex_unpack_boolean(msgpack_object obj);
mxArray* mex_unpack_positive_integer(msgpack_object obj);
mxArray* mex_unpack_negative_integer(msgpack_object obj);
mxArray* mex_unpack_double(msgpack_object obj);
mxArray* mex_unpack_raw(msgpack_object obj);
mxArray* mex_unpack_nil(msgpack_object obj);
mxArray* mex_unpack_map(msgpack_object obj);
mxArray* mex_unpack_array(msgpack_object obj);


static std::map<int, mxArray* (*)(msgpack_object obj)> unpackMap;

mxArray* mex_unpack_boolean(msgpack_object obj) {
  return mxCreateLogicalScalar(obj.via.boolean);
}

mxArray* mex_unpack_positive_integer(msgpack_object obj) {
  mxArray *ret = mxCreateNumericMatrix(1,1, mxUINT64_CLASS, mxREAL);
  uint64_t *ptr = (uint64_t *)mxGetPr(ret);
  *ptr = obj.via.u64;
  return ret;
}

mxArray* mex_unpack_negative_integer(msgpack_object obj) {
  mxArray *ret = mxCreateNumericMatrix(1,1, mxINT64_CLASS, mxREAL);
  int64_t *ptr = (int64_t *)mxGetPr(ret);
  *ptr = obj.via.i64;
  return ret;
}

mxArray* mex_unpack_double(msgpack_object obj) {
  mxArray* ret = mxCreateDoubleMatrix(1,1, mxREAL);
  double *ptr = (double *)mxGetPr(ret);
  *ptr = obj.via.dec;
  return ret;
}

mxArray* mex_unpack_raw(msgpack_object obj) {
  const char* field = (const char*)mxCalloc(obj.via.raw.size, sizeof(mxUINT8_CLASS));
  memcpy((char*)field, obj.via.raw.ptr, obj.via.raw.size);
  mxFree((void*)field);
  return mxCreateString(field);
}

mxArray* mex_unpack_nil(msgpack_object obj) {
  return mxCreateCellArray(0,0);
}

mxArray* mex_unpack_map(msgpack_object obj) {
  uint32_t nfields = obj.via.map.size;
  const char *field_name[nfields];
  for (uint32_t i = 0; i < nfields; i++) {
    struct msgpack_object_kv obj_kv = obj.via.map.ptr[i];
    if (obj_kv.key.type == MSGPACK_OBJECT_RAW) {
      field_name[i] = (const char*)mxCalloc(obj_kv.key.via.raw.size, sizeof(mxUINT8_CLASS));
      memcpy((char*)field_name[i], obj_kv.key.via.raw.ptr, obj_kv.key.via.raw.size);
    }
  }
  mxArray *ret = mxCreateStructMatrix(1, 1, obj.via.map.size, field_name);
  for (uint32_t i = 0; i < nfields; i++) {
    int ifield = mxGetFieldNumber(ret, field_name[i]);
    msgpack_object ob = obj.via.map.ptr[i].val;

    std::map<int, mxArray* (*)(msgpack_object obj)>::iterator 
        iUnpackMap = unpackMap.find(ob.type);
  
    if (iUnpackMap == unpackMap.end())
      mexErrMsgTxt("Unknown unpack function argument");
  
    if ( debug ) std::cout << msgpack_type[ob.type] << std::endl;
    mxSetFieldByNumber(ret, 0, ifield, (iUnpackMap->second)(ob));
  }
  for (uint32_t i = 0; i < nfields; i++)
    mxFree((void *)field_name[i]);
  return ret;
}

mxArray* mex_unpack_array(msgpack_object obj) {
  mxArray *ret = mxCreateCellMatrix(obj.via.array.size, 1);
  for (int i = 0; i < obj.via.array.size; i++) {
    msgpack_object ob = obj.via.array.ptr[i];

    std::map<int, mxArray* (*)(msgpack_object obj)>::iterator 
        iUnpackMap = unpackMap.find(ob.type);
  
    if (iUnpackMap == unpackMap.end())
      mexErrMsgTxt("Unknown unpack function argument");
  
    if ( debug ) std::cout << msgpack_type[ob.type] << std::endl;
    mxSetCell(ret, i, (iUnpackMap->second)(ob));
  }
  return ret;
}

void mex_unpack(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) 
{
//  if ( debug ) std::cout << "unpack "<< std::endl;
  const char *str = (const char*)mxGetPr(prhs[0]);
  size_t size = mxGetM(prhs[0]) * mxGetN(prhs[0]);

  /* deserializes it. */
  msgpack_unpacked msg;
  msgpack_unpacked_init(&msg);
  bool success = msgpack_unpack_next(&msg, str, size, NULL);
  if (!success) 
    mexErrMsgTxt("unpack error");

  /* prints the deserialized object. */
  msgpack_object obj = msg.data;
  std::map<int, mxArray* (*)(msgpack_object obj)>::iterator 
      iUnpackMap = unpackMap.find(obj.type);

  if (iUnpackMap == unpackMap.end())
    mexErrMsgTxt("Unknown unpack function argument");

  if ( debug ) std::cout << msgpack_type[obj.type] << std::endl;
  plhs[0] = (iUnpackMap->second)(obj);

}

void mex_pack(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) 
{
//  if ( debug ) std::cout << "pack "<< std::endl;
  /* creates buffer and serializer instance. */
  msgpack_sbuffer* buffer = msgpack_sbuffer_new();
  msgpack_packer* pk = msgpack_packer_new(buffer, msgpack_sbuffer_write);
 
//  msgpack_pack_true(pk);
//  /* serializes ["Hello", "MessagePack"]. */
  msgpack_pack_array(pk, 2);
  msgpack_pack_int(pk, 3455);
  msgpack_pack_int(pk, 55);
//  msgpack_pack_raw(pk, 5);
//  msgpack_pack_raw_body(pk, "Hello", 5);
//  msgpack_pack_raw(pk, 11);
//  msgpack_pack_raw_body(pk, "MessagePack", 11);

  plhs[0] = mxCreateNumericMatrix(1, buffer->size, mxUINT8_CLASS, mxREAL);
  memcpy(mxGetPr(plhs[0]), buffer->data, buffer->size * sizeof(uint8_t));

  /* cleaning */
  msgpack_sbuffer_free(buffer);
  msgpack_packer_free(pk);

  
}

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
  // Init Method Map
  static std::map<std::string, void (*)(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])> funcMap;
  funcMap["pack"] = mex_pack;
  funcMap["unpack"] = mex_unpack;

  // Init msgpack type string
  msgpack_type[MSGPACK_OBJECT_NIL] = "MSGPACK_OBJECT_NIL";
  msgpack_type[MSGPACK_OBJECT_BOOLEAN] = "MSGPACK_OBJECT_BOOLEAN";
  msgpack_type[MSGPACK_OBJECT_POSITIVE_INTEGER] = "MSGPACK_OBJECT_POSITIVE_INTEGER";
  msgpack_type[MSGPACK_OBJECT_NEGATIVE_INTEGER] = "MSGPACK_OBJECT_NEGATIVE_INTEGER";
  msgpack_type[MSGPACK_OBJECT_DOUBLE] = "MSGPACK_OBJECT_DOUBLE";
  msgpack_type[MSGPACK_OBJECT_RAW] = "MSGPACK_OBJECT_RAW";
  msgpack_type[MSGPACK_OBJECT_ARRAY] = "MSGPACK_OBJECT_ARRAY";
  msgpack_type[MSGPACK_OBJECT_MAP] = "MSGPACK_OBJECT_MAP";

  // Init unpack functions Map
  unpackMap[MSGPACK_OBJECT_NIL] = mex_unpack_nil;
  unpackMap[MSGPACK_OBJECT_BOOLEAN] = mex_unpack_boolean;
  unpackMap[MSGPACK_OBJECT_POSITIVE_INTEGER] = mex_unpack_positive_integer;
  unpackMap[MSGPACK_OBJECT_NEGATIVE_INTEGER] = mex_unpack_negative_integer;
  unpackMap[MSGPACK_OBJECT_DOUBLE] = mex_unpack_double;
  unpackMap[MSGPACK_OBJECT_RAW] = mex_unpack_raw;
  unpackMap[MSGPACK_OBJECT_ARRAY] = mex_unpack_array;
  unpackMap[MSGPACK_OBJECT_MAP] = mex_unpack_map; 


  if ((nrhs < 1) || (!mxIsChar(prhs[0])))
    mexErrMsgTxt("Need to input string argument");
  std::string fname(mxArrayToString(prhs[0]));
  
  std::map<std::string, void (*)(int nlhs, mxArray *plhs[],
    int nrhs, const mxArray *prhs[])>::iterator iFuncMap = funcMap.find(fname);

  if (iFuncMap == funcMap.end())
    mexErrMsgTxt("Unknown function argument");

  (iFuncMap->second)(nlhs, plhs, nrhs-1, prhs+1);
}

