/*
  x = field_occupancy(im);

  Matlab 7.4 MEX file to compute field occupancy.

  Compile with:
  mex -O field_occupancy.cc

  Author: Daniel D. Lee <ddlee@seas.upenn.edu>, 6/09
  Modified: Yida Zhang <yida@seas.upenn.edu>, 12/11

*/

#include "mex.h"
#include "math.h"
#include "stdlib.h"

typedef unsigned char uint8;

uint8 colorBall = 0x01;
uint8 colorField = 0x08;
uint8 colorWhite = 0x10;

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
  // Check arguments
  if ((nrhs < 1)  || !((mxGetClassID(prhs[0]) == mxUINT8_CLASS)))
    mexErrMsgTxt("Need uint8 input image.");

  uint8 *im_ptr = (uint8 *) mxGetData(prhs[0]);
  int ni = mxGetM(prhs[0]);
  int nj = mxGetN(prhs[0]);
  const int nRegions = ni;

  int count[nRegions];
  int flag[nRegions];
  for (int i = 0; i < nRegions; i++) {
    count[i] = 0;
    flag[i] = 0;
  }

  // Scan vertical lines:
  for (int i = 0; i < ni; i++) {
    int iRegion = nRegions*i/ni;
    uint8 *im_row = im_ptr + i;
    for (int j = 0; j < nj; j++) {
      uint8 label = *im_row;
      if ((label & colorField) || (label & colorBall) || (label & colorWhite)) {
        count[iRegion]++;
      }
      im_row += ni;
    }
  }
  
  // Evaluate bound
  for (int i = 0; i < nRegions; i++){
    int pxIdx = (nj - count[i] + 1) * ni + i;
    uint8 label = *(im_ptr + pxIdx);
    if ((label & colorField) || (label & colorBall) || (label & colorWhite))
      flag[i] = 1;
    else {
      //printf("Seeking\n");
      int j = nj - count[i] + 1;
      for (; j < nj; j++){
        int searchIdx = j * ni + i;
        uint8 searchLabel = *(im_ptr + searchIdx);
        if ((searchLabel & colorField) || (searchLabel & colorBall) || (searchLabel & colorWhite))
            break;
      }
      count[i] = nj - j;
      flag[i] = 1;
    }
  }

  plhs[0] = mxCreateDoubleMatrix(1, nRegions, mxREAL);
  //plhs[1] = mxCreateDoubleMatrix(1, nRegions, mxREAL);
  for (int i = 0; i < nRegions; i++){
    mxGetPr(plhs[0])[i] = count[i];
    //mxGetPr(plhs[1])[i] = flag[i];
  }
}
