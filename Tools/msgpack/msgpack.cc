#include <iostream>
#include <string>
#include <map>
#include <vector>
#include <stdio.h>
#include <msgpack.hpp>
#include "mex.h"

using namespace std;

void mex_unpack(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) 
{
  std::cout << "unpack "<< std::endl;
  const char *str = mxArrayToString(prhs[0]);
  size_t size = mxGetScalar(prhs[1]);
  std::cout << str << std::endl;
  std::cout << size << std::endl;
  msgpack_unpacked msg;
  msgpack_unpacked_init(&msg);
  bool success = msgpack_unpack_next(&msg, str, size, NULL);

  msgpack_object obj = msg.data;
  printf("%d\n", obj.type);

}

void mex_pack(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) 
{
  std::cout << "pack "<< std::endl;
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

