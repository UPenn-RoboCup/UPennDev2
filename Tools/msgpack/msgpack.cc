#include <iostream>
#include <string>
#include <map>
#include <vector>
#include <stdio.h>
#include <unistd.h>
#include <msgpack.h>

#include "mex.h"

using namespace std;

void mex_unpack(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) 
{
//  std::cout << "unpack "<< std::endl;
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
    std::cout << "MSGPACK_OBJECT_BOOLEAN" << std::endl;
    plhs[0] = mxCreateLogicalScalar(obj.via.boolean);
  }
  else if (obj.type == MSGPACK_OBJECT_POSITIVE_INTEGER) {
    std::cout << "MSGPACK_OBJECT_POSITIVE_INTEGER" << std::endl;
    plhs[0] = mxCreateNumericMatrix(1,1, mxUINT64_CLASS, mxREAL);
    uint64_t *ptr = (uint64_t *)mxGetPr(plhs[0]);
    *ptr = obj.via.u64;
  }
  else if (obj.type == MSGPACK_OBJECT_NEGATIVE_INTEGER) {
    std::cout << "MSGPACK_OBJECT_NEGATIVE_INTEGER" << std::endl;
    plhs[0] = mxCreateNumericMatrix(1,1, mxINT64_CLASS, mxREAL);
    int64_t *ptr = (int64_t *)mxGetPr(plhs[0]);
    *ptr = obj.via.i64;
  }
  else if (obj.type == MSGPACK_OBJECT_DOUBLE) {
    std::cout << "MSGPACK_OBJECT_DOUBLE" << std::endl;
    plhs[0] = mxCreateDoubleMatrix(1,1, mxREAL);
    double *ptr = (double *)mxGetPr(plhs[0]);
    *ptr = obj.via.dec;
  }
  else if (obj.type == MSGPACK_OBJECT_RAW) {
    std::cout << "MSGPACK_OBJECT_RAW" << std::endl;
    plhs[0] = mxCreateString(obj.via.raw.ptr);
  }
  else if (obj.type == MSGPACK_OBJECT_ARRAY) {
    std::cout << "MSGPACK_OBJECT_ARRAY" << std::endl;
  }
  else if (obj.type == MSGPACK_OBJECT_MAP)
    std::cout << "MSGPACK_OBJECT_MAP" << std::endl;
  else
    plhs[0] = mxCreateCellArray(0,0);    
}

void mex_pack(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) 
{
//  std::cout << "pack "<< std::endl;
  /* creates buffer and serializer instance. */
  msgpack_sbuffer* buffer = msgpack_sbuffer_new();
  msgpack_packer* pk = msgpack_packer_new(buffer, msgpack_sbuffer_write);
 
//  /* serializes ["Hello", "MessagePack"]. */
  msgpack_pack_array(pk, 2);
  msgpack_pack_int(pk, 3455);
//  msgpack_pack_raw(pk, 5);
//  msgpack_pack_raw_body(pk, "Hello", 5);
  msgpack_pack_raw(pk, 11);
  msgpack_pack_raw_body(pk, "MessagePack", 11);

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

