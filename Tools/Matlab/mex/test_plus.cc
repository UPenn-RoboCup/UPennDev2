/*
  x = test_plus(a, b);
  
  Matlab 7.4 MEX file

  Author: Jinwook HUh <jinwookh@seas.upenn.edu>, 6/09
*/

#include "mex.h"
#include "RegionProps.h"

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
  //uint8 mask = 0x01;

  // Check arguments
  if ((nrhs < 1)) //  || !((mxGetClassID(prhs[0]) == mxUINT8_CLASS)))
    mexErrMsgTxt("Need two variables.");

  double a = mxGetScalar(prhs[0]);
  double b = mxGetScalar(prhs[1]);

  double result = a+ b;
  plhs[0] = mxCreateDoubleMatrix(1, 1, mxREAL);
  double *v = (double*)mxGetData(plhs[0]);  
  (*v)= result;  
}
