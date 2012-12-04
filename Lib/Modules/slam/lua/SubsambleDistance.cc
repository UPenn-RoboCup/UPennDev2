
#ifdef __cplusplus
extern "C" {
#endif

#include "lua.h"
#include "lualib.h"
#include "lauxlib.h"

#ifdef __cplusplus
}
#endif

#include <inttypes.h>
#include <string.h>
#include <math.h>
#include "Timer.hh"


#define MAX_NUM_POINTS 1081
double idxs[MAX_NUM_POINTS];

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
  if (nrhs != 3)
    mexErrMsgTxt("need 3 arguments: rs, angleStep, maxdist");
    
    
  double * rs = mxGetPr(prhs[0]);
  double astep = mxGetPr(prhs[1])[0];
  double maxdist = mxGetPr(prhs[2])[0];
  int nrs = mxGetNumberOfElements(prhs[0]);
  
  if (nrs > MAX_NUM_POINTS)
    mexErrMsgTxt("too many points");
  
  double costh = cos(astep);
  double nextBreak = maxdist;
  double currDist  = 0;
  
  double * trs = rs;
  
  double a = *trs++;
  double a2 = a*a;
  double b;
  double b2;
  double c2;
  trs++;
  
  int nBreaks = 0;
  
  for (int ii=1; ii<nrs; ii++)
  {
    b = *trs++;
    b2 = b*b;
    c2 = a2 + b2 - 2*a*b*costh;
    currDist+=sqrt(c2);
    
    if (currDist > nextBreak)
    {
      idxs[nBreaks++] = ii;
      nextBreak+=maxdist;
    }
  
    a=b;
    a2=b2;
  }
  
  
  plhs[0] = mxCreateDoubleMatrix(nBreaks,1,mxREAL);
  memcpy(mxGetData(plhs[0]),idxs,nBreaks*sizeof(double));

}
