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

mxArray* mex_unpack_boolean(msgpack_object obj);
mxArray* mex_unpack_positive_integer(msgpack_object obj);
mxArray* mex_unpack_negative_integer(msgpack_object obj);
mxArray* mex_unpack_double(msgpack_object obj);
mxArray* mex_unpack_raw(msgpack_object obj);
mxArray* mex_unpack_nil(msgpack_object obj);
mxArray* mex_unpack_map(msgpack_object obj);
mxArray* mex_unpack_array(msgpack_object obj);

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

    if (ob.type == MSGPACK_OBJECT_BOOLEAN) {
      if ( debug ) std::cout << "MSGPACK_OBJECT_BOOLEAN" << std::endl;
      mxSetFieldByNumber(ret, 0, ifield, mex_unpack_boolean(ob));
    }
    else if (ob.type == MSGPACK_OBJECT_POSITIVE_INTEGER) {
      if ( debug ) std::cout << "MSGPACK_OBJECT_POSITIVE_INTEGER" << std::endl;
      mxSetFieldByNumber(ret, 0, ifield, mex_unpack_positive_integer(ob));
    }
    else if (ob.type == MSGPACK_OBJECT_NEGATIVE_INTEGER) {
      if ( debug ) std::cout << "MSGPACK_OBJECT_NEGATIVE_INTEGER" << std::endl;
      mxSetFieldByNumber(ret, 0, ifield, mex_unpack_negative_integer(ob));
    }
    else if (ob.type == MSGPACK_OBJECT_DOUBLE) {
      if ( debug ) std::cout << "MSGPACK_OBJECT_DOUBLE" << std::endl;
      mxSetFieldByNumber(ret, 0, ifield, mex_unpack_double(ob));
    }
    else if (ob.type == MSGPACK_OBJECT_RAW) {
      if ( debug ) std::cout << "MSGPACK_OBJECT_RAW" << std::endl;
      mxSetFieldByNumber(ret, 0, ifield, mex_unpack_raw(ob));
    }
    else if (ob.type == MSGPACK_OBJECT_ARRAY) {
      if ( debug ) std::cout << "MSGPACK_OBJECT_ARRAY" << std::endl;
      mxSetFieldByNumber(ret, 0, ifield, mex_unpack_array(ob));
    }
    else if (ob.type == MSGPACK_OBJECT_MAP) {
      if ( debug ) std::cout << "MSGPACK_OBJECT_MAP" << std::endl;
      mxSetFieldByNumber(ret, 0, ifield, mex_unpack_map(ob));
    }
    else
      mxSetFieldByNumber(ret, 0, ifield, mex_unpack_nil(ob));
  }
  for (uint32_t i = 0; i < nfields; i++)
    mxFree((void *)field_name[i]);
  return ret;
}

mxArray* mex_unpack_array(msgpack_object obj) {
  mxArray *ret = mxCreateCellMatrix(obj.via.array.size, 1);
  for (int i = 0; i < obj.via.array.size; i++) {
    msgpack_object ob = obj.via.array.ptr[i];
    if (ob.type == MSGPACK_OBJECT_BOOLEAN) {
      if ( debug ) std::cout << "MSGPACK_OBJECT_BOOLEAN" << std::endl;
      mxSetCell(ret, i, mex_unpack_boolean(ob));
    }
    else if (ob.type == MSGPACK_OBJECT_POSITIVE_INTEGER) {
      if ( debug ) std::cout << "MSGPACK_OBJECT_POSITIVE_INTEGER" << std::endl;
      mxSetCell(ret, i, mex_unpack_positive_integer(ob));
    }
    else if (ob.type == MSGPACK_OBJECT_NEGATIVE_INTEGER) {
      if ( debug ) std::cout << "MSGPACK_OBJECT_NEGATIVE_INTEGER" << std::endl;
      mxSetCell(ret, i, mex_unpack_negative_integer(ob));
    }
    else if (ob.type == MSGPACK_OBJECT_DOUBLE) {
      if ( debug ) std::cout << "MSGPACK_OBJECT_DOUBLE" << std::endl;
      mxSetCell(ret, i, mex_unpack_double(ob));
    }
    else if (ob.type == MSGPACK_OBJECT_RAW) {
      if ( debug ) std::cout << "MSGPACK_OBJECT_RAW" << std::endl;
      mxSetCell(ret, i, mex_unpack_raw(ob));
    }
    else if (ob.type == MSGPACK_OBJECT_ARRAY) {
      if ( debug ) std::cout << "MSGPACK_OBJECT_ARRAY" << std::endl;
      mxSetCell(ret, i, mex_unpack_array(ob));
    }
    else if (ob.type == MSGPACK_OBJECT_MAP) {
      if ( debug ) std::cout << "MSGPACK_OBJECT_MAP" << std::endl;
      mxSetCell(ret, i, mex_unpack_map(ob));
    }
    else
      mxSetCell(ret, i, mex_unpack_nil(ob));
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

  if (obj.type == MSGPACK_OBJECT_BOOLEAN) {
    if ( debug ) std::cout << "MSGPACK_OBJECT_BOOLEAN" << std::endl;
    plhs[0] = mex_unpack_boolean(obj);
  }
  else if (obj.type == MSGPACK_OBJECT_POSITIVE_INTEGER) {
    if ( debug ) std::cout << "MSGPACK_OBJECT_POSITIVE_INTEGER" << std::endl;
    plhs[0] = mex_unpack_positive_integer(obj);
  }
  else if (obj.type == MSGPACK_OBJECT_NEGATIVE_INTEGER) {
    if ( debug ) std::cout << "MSGPACK_OBJECT_NEGATIVE_INTEGER" << std::endl;
    plhs[0] = mex_unpack_negative_integer(obj);
  }
  else if (obj.type == MSGPACK_OBJECT_DOUBLE) {
    if ( debug ) std::cout << "MSGPACK_OBJECT_DOUBLE" << std::endl;
    plhs[0] = mex_unpack_double(obj);
  }
  else if (obj.type == MSGPACK_OBJECT_RAW) {
    if ( debug ) std::cout << "MSGPACK_OBJECT_RAW" << std::endl;
    plhs[0] = mex_unpack_raw(obj);
  }
  else if (obj.type == MSGPACK_OBJECT_ARRAY) {
    if ( debug ) std::cout << "MSGPACK_OBJECT_ARRAY" << std::endl;
    plhs[0] = mex_unpack_array(obj);
  }
  else if (obj.type == MSGPACK_OBJECT_MAP) {
    if ( debug ) std::cout << "MSGPACK_OBJECT_MAP" << std::endl;
    plhs[0] = mex_unpack_map(obj);
  }
  else
    plhs[0] = mex_unpack_nil(obj);    
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
  static std::map<std::string, void (*)(int nlhs, mxArray *plhs[],
    int nrhs, const mxArray *prhs[])> funcMap;
  funcMap["pack"] = mex_pack;
  funcMap["unpack"] = mex_unpack;

  if ((nrhs < 1) || (!mxIsChar(prhs[0])))
    mexErrMsgTxt("Need to input string argument");
  std::string fname(mxArrayToString(prhs[0]));
  
  std::map<std::string, void (*)(int nlhs, mxArray *plhs[],
    int nrhs, const mxArray *prhs[])>::iterator iFuncMap = funcMap.find(fname);

  if (iFuncMap == funcMap.end())
    mexErrMsgTxt("Unknown function argument");

  (iFuncMap->second)(nlhs, plhs, nrhs-1, prhs+1);
}

