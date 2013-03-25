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
  std::cout << "unpack "<< std::endl;
  const char *str = (const char*)mxGetPr(prhs[0]);
  size_t size = mxGetM(prhs[0]) * mxGetN(prhs[0]);

  /* deserializes it. */
  msgpack_unpacked msg;
  msgpack_unpacked_init(&msg);
  bool success = msgpack_unpack_next(&msg, str, size, NULL);
// 
  /* prints the deserialized object. */
  msgpack_object obj = msg.data;
  msgpack_object_print(stdout, obj);  /*=> ["Hello", "MessagePack"] */
  printf("%d", obj.type);
}

void mex_pack(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) 
{
  std::cout << "pack "<< std::endl;
  /* creates buffer and serializer instance. */
  msgpack_sbuffer* buffer = msgpack_sbuffer_new();
  msgpack_packer* pk = msgpack_packer_new(buffer, msgpack_sbuffer_write);
 
  /* serializes ["Hello", "MessagePack"]. */
  msgpack_pack_array(pk, 2);
  msgpack_pack_raw(pk, 5);
  msgpack_pack_raw_body(pk, "Hello", 5);
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

