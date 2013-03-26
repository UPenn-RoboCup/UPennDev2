#include <iostream>
#include <string>
#include <map>
#include <vector>
#include <stdio.h>
#include <unistd.h>
#include <msgpack.h>

#include "mex.h"

using namespace std;

bool debug = false;
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
static std::map<int, void (*)(msgpack_packer *pk, 
                int nrhs, const mxArray *prhs)> packMap;

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
  mwSize dims[] = {1, obj.via.raw.size/2};
  mxArray* ret = mxCreateCharArray(2, dims);
  char *ptr = (char*)mxGetPr(ret); 
  memcpy(ptr, obj.via.raw.ptr, obj.via.raw.size * sizeof(uint8_t));
  return ret;
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
      field_name[i] = (const char*)mxCalloc(obj_kv.key.via.raw.size, sizeof(uint8_t));
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

void mex_pack_single(msgpack_packer *pk, int nrhs, const mxArray *prhs) {
  size_t nElements = mxGetNumberOfElements(prhs);
  float *data = (float*)mxGetPr(prhs);
  if (nElements > 1) msgpack_pack_array(pk, nElements);
  for (int i = 0; i < nElements; i++) {
    msgpack_pack_float(pk, data[i]);
  }
}

void mex_pack_double(msgpack_packer *pk, int nrhs, const mxArray *prhs) {
  size_t nElements = mxGetNumberOfElements(prhs);
  double *data = mxGetPr(prhs);
  if (nElements > 1) msgpack_pack_array(pk, nElements);
  for (int i = 0; i < nElements; i++) {
    msgpack_pack_double(pk, data[i]);
  }
}

void mex_pack_int8(msgpack_packer *pk, int nrhs, const mxArray *prhs) {
  size_t nElements = mxGetNumberOfElements(prhs);
  int8_t *data = (int8_t*)mxGetPr(prhs);
  if (nElements > 1) msgpack_pack_array(pk, nElements);
  for (int i = 0; i < nElements; i++) {
    msgpack_pack_int8(pk, data[i]);
  }
}

void mex_pack_uint8(msgpack_packer *pk, int nrhs, const mxArray *prhs) {
  size_t nElements = mxGetNumberOfElements(prhs);
  uint8_t *data = (uint8_t*)mxGetPr(prhs);
  if (nElements > 1) msgpack_pack_array(pk, nElements);
  for (int i = 0; i < nElements; i++) {
    msgpack_pack_uint8(pk, data[i]);
  }
}

void mex_pack_int16(msgpack_packer *pk, int nrhs, const mxArray *prhs) {
  size_t nElements = mxGetNumberOfElements(prhs);
  int16_t *data = (int16_t*)mxGetPr(prhs);
  if (nElements > 1) msgpack_pack_array(pk, nElements);
  for (int i = 0; i < nElements; i++) {
    msgpack_pack_int16(pk, data[i]);
  }
}

void mex_pack_uint16(msgpack_packer *pk, int nrhs, const mxArray *prhs) {
  size_t nElements = mxGetNumberOfElements(prhs);
  uint16_t *data = (uint16_t*)mxGetPr(prhs);
  if (nElements > 1) msgpack_pack_array(pk, nElements);
  for (int i = 0; i < nElements; i++) {
    msgpack_pack_uint16(pk, data[i]);
  }
}

void mex_pack_int32(msgpack_packer *pk, int nrhs, const mxArray *prhs) {
  size_t nElements = mxGetNumberOfElements(prhs);
  int32_t *data = (int32_t*)mxGetPr(prhs);
  if (nElements > 1) msgpack_pack_array(pk, nElements);
  for (int i = 0; i < nElements; i++) {
    msgpack_pack_int32(pk, data[i]);
  }
}

void mex_pack_uint32(msgpack_packer *pk, int nrhs, const mxArray *prhs) {
  size_t nElements = mxGetNumberOfElements(prhs);
  uint32_t *data = (uint32_t*)mxGetPr(prhs);
  if (nElements > 1) msgpack_pack_array(pk, nElements);
  for (int i = 0; i < nElements; i++) {
    msgpack_pack_uint32(pk, data[i]);
  }
}

void mex_pack_int64(msgpack_packer *pk, int nrhs, const mxArray *prhs) {
  size_t nElements = mxGetNumberOfElements(prhs);
  int64_t *data = (int64_t*)mxGetPr(prhs);
  if (nElements > 1) msgpack_pack_array(pk, nElements);
  for (int i = 0; i < nElements; i++) {
    msgpack_pack_int64(pk, data[i]);
  }
}

void mex_pack_uint64(msgpack_packer *pk, int nrhs, const mxArray *prhs) {
  size_t nElements = mxGetNumberOfElements(prhs);
  uint64_t *data = (uint64_t*)mxGetPr(prhs);
  if (nElements > 1) msgpack_pack_array(pk, nElements);
  for (int i = 0; i < nElements; i++) {
    msgpack_pack_uint64(pk, data[i]);
  }
}

void mex_pack_logical(msgpack_packer *pk, int nrhs, const mxArray *prhs) {
  size_t nElements = mxGetNumberOfElements(prhs);
  bool *data = mxGetLogicals(prhs);
  if (nElements > 1) msgpack_pack_array(pk, nElements);
  for (int i = 0; i < nElements; i++)
    if (data[i]) msgpack_pack_true(pk);
    else msgpack_pack_false(pk);
}

void mex_pack_char(msgpack_packer *pk, int nrhs, const mxArray *prhs) {
  size_t nElements = mxGetNumberOfElements(prhs);
  uint8_t *data = (uint8_t*)mxGetPr(prhs); 
  // matlab char type is actually uint16 -> 2 * uint8
  msgpack_pack_raw(pk, nElements * 2);
  msgpack_pack_raw_body(pk, data, nElements * 2);
}

void mex_pack_cell(msgpack_packer *pk, int nrhs, const mxArray *prhs) {
  size_t nElements = mxGetNumberOfElements(prhs);
  if (nElements > 1) msgpack_pack_array(pk, nElements);
  for (int i = 0; i < nElements; i++) {
    mxArray* pm = mxGetCell(prhs, i);
    std::map<int, void (*)(msgpack_packer *pk, int nrhs, const mxArray *prhs)>::iterator
       iPackMap = packMap.find(mxGetClassID(pm));

    if (iPackMap == packMap.end())
      mexErrMsgTxt("Unknown pack function argument");

    (iPackMap->second)(pk, nrhs, pm);
  }
}

void mex_pack_struct(msgpack_packer *pk, int nrhs, const mxArray *prhs) {
  size_t nField = mxGetNumberOfFields(prhs);
  if (nField > 1) msgpack_pack_map(pk, nField);
  for (int i = 0; i < nField; i++) {
    const char* fname = mxGetFieldNameByNumber(prhs, i);
    size_t fnameLen = strlen(fname);
    msgpack_pack_raw(pk, fnameLen);
    msgpack_pack_raw_body(pk, fname, fnameLen);
    int ifield = mxGetFieldNumber(prhs, fname);
    mxArray* pm = mxGetFieldByNumber(prhs, 0, ifield);

    std::map<int, void (*)(msgpack_packer *pk, int nrhs, const mxArray *prhs)>::iterator
       iPackMap = packMap.find(mxGetClassID(pm));

    if (iPackMap == packMap.end())
      mexErrMsgTxt("Unknown pack function argument");

    (iPackMap->second)(pk, nrhs, pm);

  }
}

void mex_pack(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  /* creates buffer and serializer instance. */
  msgpack_sbuffer* buffer = msgpack_sbuffer_new();
  msgpack_packer* pk = msgpack_packer_new(buffer, msgpack_sbuffer_write);

  for (int i = 0; i < nrhs; i ++) {
    std::map<int, void (*)(msgpack_packer *pk, int nrhs, const mxArray *prhs)>::iterator
       iPackMap = packMap.find(mxGetClassID(prhs[i]));

    if (iPackMap == packMap.end())
      mexErrMsgTxt("Unknown pack function argument");

    (iPackMap->second)(pk, nrhs, prhs[i]);
  }

  plhs[0] = mxCreateNumericMatrix(1, buffer->size, mxUINT8_CLASS, mxREAL);
  memcpy(mxGetPr(plhs[0]), buffer->data, buffer->size * sizeof(uint8_t));

  /* cleaning */
  msgpack_sbuffer_free(buffer);
  msgpack_packer_free(pk);
}

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
  // Init Method Map
  static std::map<std::string, void (*)(int nlhs, mxArray *plhs[], 
      int nrhs, const mxArray *prhs[])> funcMap;
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

//  packMap[mxUNKNOWN_CLASS] = mex_pack_nil;
  packMap[mxCELL_CLASS] = mex_pack_cell;
  packMap[mxSTRUCT_CLASS] = mex_pack_struct;
  packMap[mxLOGICAL_CLASS] = mex_pack_logical;
  packMap[mxCHAR_CLASS] = mex_pack_char;
//  packMap[mxVOID_CLASS] =
  packMap[mxDOUBLE_CLASS] = mex_pack_double;
  packMap[mxSINGLE_CLASS] = mex_pack_single;
  packMap[mxINT8_CLASS] = mex_pack_int8;
  packMap[mxUINT8_CLASS] = mex_pack_uint8;
  packMap[mxINT16_CLASS] = mex_pack_int16;
  packMap[mxUINT16_CLASS] = mex_pack_uint16;
  packMap[mxINT32_CLASS] = mex_pack_int32;
  packMap[mxUINT32_CLASS] = mex_pack_uint32;
  packMap[mxINT64_CLASS] = mex_pack_int64;
  packMap[mxUINT64_CLASS] = mex_pack_uint64;
//  packMap[mxFUNCTION_CLASS] =


  if ((nrhs < 1) || (!mxIsChar(prhs[0])))
    mexErrMsgTxt("Need to input string argument");
  std::string fname(mxArrayToString(prhs[0]));
  
  std::map<std::string, void (*)(int nlhs, mxArray *plhs[],
    int nrhs, const mxArray *prhs[])>::iterator iFuncMap = funcMap.find(fname);

  if (iFuncMap == funcMap.end())
    mexErrMsgTxt("Unknown function argument");

  (iFuncMap->second)(nlhs, plhs, nrhs-1, prhs+1);
}

