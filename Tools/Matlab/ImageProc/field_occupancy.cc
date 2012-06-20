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
#include "stdio.h"

#include <iostream>

typedef unsigned char uint8;

uint8 colorBall = 0x01;
uint8 colorField = 0x08;
uint8 colorWhite = 0x10;

inline bool isFree(uint8 label) 
{
  return (label & colorField) || (label & colorBall) || (label & colorWhite);
}

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
  // Check arguments
  if ((nrhs < 1)  || !((mxGetClassID(prhs[0]) == mxUINT8_CLASS)))
    mexErrMsgTxt("Need uint8 input image.");

  uint8 *im_ptr = (uint8 *) mxGetData(prhs[0]);
  int ni = mxGetM(prhs[0]);
  int nj = mxGetN(prhs[0]);
  const int nRegions = ni;

  int blockpos[nj];
  int blockcluster[nj];
  int blockclusterE[nj];
  int countup[nRegions];
  int countdown[nRegions];
  int count[nRegions];
  int flag[nRegions] ;

  for (int i = 0; i < nRegions; i++) {
    count[i] = 0;
    flag[i] = 0;
    countdown[i] = 0;
  }


  // Scan vertical lines: Uphalf
  int nBlocks = 0, nBlockClusters = 0;
  int lastOb = 0;
  bool inOb = false;
  for (int i = 0; i < ni; i++) {
    int iRegion = nRegions*i/ni;
    uint8 *im_row = im_ptr + i;
    nBlocks = 0;
    nBlockClusters = 0;
    for (int j = 0; j < nj; j++) {
      blockpos[j] = 0;
      blockcluster[j] = 0;
      blockclusterE[j] = 0;
    }
    for (int j = 0; j < nj; j++) {
      uint8 label = *im_row;
      if (!isFree(label)) {
        blockpos[nBlocks] = j;
        if (nBlocks == 0) {
          inOb = true;
//          std::cout << i << ' ' << "new ob" << std::endl;
          blockcluster[nBlockClusters] = j;
          nBlockClusters++;
        }
        else if ((blockpos[nBlocks] - blockpos[nBlocks - 1]) > 5) {
//          std::cout  << i << ' ' <<"more ob" << std::endl;
          inOb = true;
          blockcluster[nBlockClusters] = j;
          nBlockClusters++;
        }
        nBlocks++;
      }
      im_row += ni; 
      if (i == 50) {
        std::cout << nBlockClusters << ' ' << blockcluster[nBlockClusters-1] << std::endl;
      }
    }


    // no black pixels found, return type 1
    if (nBlocks < 0.05 * nj) {
//      std::cout << "t1" << std::endl;
      flag[i] = 1;
      count[i] = nj - 1;
      continue;
    }
    if (nBlocks > 0.95 * nj) {
//      std::cout << "t2" << std::endl;
      flag[i] = 3;
      count[i] = 0;
      continue;
    }
  }

  plhs[0] = mxCreateDoubleMatrix(1, nRegions, mxREAL);
  plhs[1] = mxCreateDoubleMatrix(1, nRegions, mxREAL);
  for (int i = 0; i < nRegions; i++){
    mxGetPr(plhs[0])[i] = count[i];
    mxGetPr(plhs[1])[i] = flag[i];
  }
}
