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
  int countpos[nRegions];
  for (int i = 0; i < nRegions; i++) {
    count[i] = 0;
    countpos[i] = 0;
  }

  // Scan vertical lines:
  float aveval = 0; // average position based on value 
  float aveidx = 0; // average position based on index
  for (int i = 0; i < ni; i++) {
    int iRegion = nRegions*i/ni;
    uint8 *im_row = im_ptr + i;
    for (int j = 0; j < nj; j++) {
      uint8 label = *im_row;
      if ((label & colorField) || (label & colorBall) || (label & colorWhite)) {
        count[iRegion]++;
        countpos[iRegion] += (j + 1);
      }
      im_row += ni;
    }
    if (count[i] != 0){
      aveval = nj - count[i]/2;
      aveidx = countpos[i]/count[i];
      if (abs(aveval-aveidx) > 0.1*nj){
        // printf("recalibrate\n");
        count[i] = 0; 
        uint8 *im_row = im_ptr + i;
        for (int j = aveidx; j < nj; j++) {
          uint8 label = *im_row;
          if ((label & colorField) || (label & colorBall) || (label & colorWhite))
            count[i]++;
          im_row += ni;
        }      
      }
    }
  }
  

  plhs[0] = mxCreateDoubleMatrix(1, nRegions, mxREAL);
  for (int i = 0; i < nRegions; i++){
    mxGetPr(plhs[0])[i] = count[i];
  }
}
