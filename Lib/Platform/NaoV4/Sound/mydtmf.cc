/*
 * Mex file to test the dtmf tone detection
 * 
 * To compile:
 * mex -O mydtmf.cc
 *  
 * Jordan Brindza; <brindza@seas.upenn.edu>
 */

#include <stdio.h>

#include "mex.h"
#include "dtmf.h"

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  // Check arguments
  if (nrhs < 1) {
    mexErrMsgTxt("Need to input array.");
  }

  short *x;

  if (nrhs < 2) {
    // assumed to be actual interleaved audio signal
    x = (short *)mxGetData(prhs[0]);

    check_tone(x);
  } else if (nrhs == 2) {
    // interleave left and right audio signals
    double *l = mxGetPr(prhs[0]);
    double *r = mxGetPr(prhs[1]);
    
    int ml = mxGetM(prhs[0]);
    int nl = mxGetN(prhs[0]);
    if (ml != 1 && nl != 1) {
      mexErrMsgTxt("Input array must be 1 dimensional.");
    }
    int mr = mxGetM(prhs[1]);
    int nr = mxGetN(prhs[1]);
    if (mr != 1 && nr != 1) {
      mexErrMsgTxt("Input array must be 1 dimensional.");
    }

    if (ml != mr || nl != nr) {
      mexErrMsgTxt("input arrays must be same size.");
    }

    int n = (ml > nl) ? ml : nl;
    x = (short *)malloc(2*n*sizeof(short));
    for (int i = 0; i < n; i++) {
      x[2*i]    = (short)l[i];
      x[2*i+1]  = (short)r[i];
    }

    check_tone(x);

    if (nrhs == 2) {
      free(x);
    }
  }
}

