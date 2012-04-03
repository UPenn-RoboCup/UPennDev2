/*
  RadonTransform class
  Written by Daniel D. Lee, 06/2009
  <ddlee@seas.upenn.edu>
*/

#include "RadonTransform.h"
#include <stdlib.h>
#include <limits.h>
#include <math.h>

#ifndef PI
#define PI M_PI
#endif

RadonTransform::RadonTransform() {
  // Initialize trigonometry tables:
  for (int i = 0; i < NTH; i++) {
    //We only need 0 to Pi
    th[i] = PI*i/NTH;
    cosTable[i] = NTRIG*cos(th[i]);
    sinTable[i] = NTRIG*sin(th[i]);
  }

  clear();
}

void RadonTransform::clear() {
  countMax = 0;
  for (int ith = 0; ith < NTH; ith++) {
    for (int ir = 0; ir < NR; ir++) {
      count[ith][ir] = 0;
      lineMin[ith][ir] = INT_MAX;
      lineMax[ith][ir] = INT_MIN;
      lineSum[ith][ir] = 0;
    }
  }
}

void RadonTransform::addPixelToRay(int i, int j, int ith) {
  int ir = abs(cosTable[ith]*i + sinTable[ith]*j)/NTRIG; 
//R value: 0 to MAXR-1
//R index: 0 to NR-1
  int ir1=(ir+1)*NR/MAXR-1;
  count[ith][ir1]++;
  if (count[ith][ir1] > countMax) {
    thMax = ith;
    rMax = ir1;
    countMax = count[ith][ir1];
  }

  // Line statistics:
  int iline = (-sinTable[ith]*i + cosTable[ith]*j)/NTRIG;
  lineSum[ith][ir1] += iline;
  if (iline > lineMax[ith][ir1]) lineMax[ith][ir1] = iline;
  if (iline < lineMin[ith][ir1]) lineMin[ith][ir1] = iline;    
}

void RadonTransform::addHorizontalPixel(int i, int j) {
  for (int ith = 0; ith < NTH; ith++) {
    if (abs(sinTable[ith]) < DIAGONAL_THRESHOLD)
      continue;
    addPixelToRay(i, j, ith);
  }
}

void RadonTransform::addVerticalPixel(int i, int j) {
  for (int ith = 0; ith < NTH; ith++) {
    if (abs(cosTable[ith]) < DIAGONAL_THRESHOLD)
      continue;
    addPixelToRay(i, j, ith);
  }
}

struct LineStats &RadonTransform::getLineStats() {
  bestLine.count = countMax;
  if (countMax == 0) {
    return bestLine;
  }
//R value: 0 to MAXR-1
//R index: 0 to NR-1

  double iR = ((rMax+1)*MAXR/NR-1+.5)*cosTable[thMax];
  double jR = ((rMax+1)*MAXR/NR-1+.5)*sinTable[thMax];
  double lMean = lineSum[thMax][rMax]/countMax;
  double lMin = lineMin[thMax][rMax];
  double lMax = lineMax[thMax][rMax];

  bestLine.iMean = (iR - lMean*sinTable[thMax])/NTRIG;
  bestLine.jMean = (jR + lMean*cosTable[thMax])/NTRIG;
  bestLine.iMin = (iR - lMin*sinTable[thMax])/NTRIG;
  bestLine.jMin = (jR + lMin*cosTable[thMax])/NTRIG;
  bestLine.iMax = (iR - lMax*sinTable[thMax])/NTRIG;
  bestLine.jMax = (jR + lMax*cosTable[thMax])/NTRIG;

  return bestLine;
}


struct LineStats *RadonTransform::getMultiLineStats(){

  int max_previous_count=9999;
  int thMax=0, rMax=0, countMax=0;

  for (int i=0;i<MAXLINES;i++){
    countMax=-1;thMax=0;rMax=0;
    for (int ith = 0; ith < NTH; ith++) {
      for (int ir = 0; ir < NR; ir++) {
        if ((count[ith][ir]>countMax)
	   &&(count[ith][ir]<max_previous_count)){
	  countMax=count[ith][ir];
	  thMax=ith;rMax=ir;
	}
      }
    }
    if (countMax>0){
//R value: 0 to MAXR-1
//R index: 0 to NR-1

      double iR = ((rMax+1)*MAXR/NR-1+.5)*cosTable[thMax];
      double jR = ((rMax+1)*MAXR/NR-1+.5)*sinTable[thMax];
      double lMean = lineSum[thMax][rMax]/countMax;
      double lMin = lineMin[thMax][rMax];
      double lMax = lineMax[thMax][rMax];
      bestLines[i].count = countMax;
      bestLines[i].iMean = (iR - lMean*sinTable[thMax])/NTRIG;
      bestLines[i].jMean = (jR + lMean*cosTable[thMax])/NTRIG;
      bestLines[i].iMin = (iR - lMin*sinTable[thMax])/NTRIG;
      bestLines[i].iMax = (iR - lMax*sinTable[thMax])/NTRIG;
      bestLines[i].jMin = (jR + lMin*cosTable[thMax])/NTRIG;
      bestLines[i].jMax = (jR + lMax*cosTable[thMax])/NTRIG;
      max_previous_count=countMax;
    }
  }

  return bestLines;
}
