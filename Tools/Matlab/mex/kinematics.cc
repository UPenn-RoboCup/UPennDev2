/*
  x = kinematics(q);

  Matlab MEX file to run THOR-OP kinematics

  Compile with:
  mex -O kinematics.cc Transform.cpp THOROPKinematics.cpp -I../../../Robots/THOROP -I../../../Robots/Transform
	mex -O kinematics.cc ../../../Robots/THOROP/THOROPKinematics.cpp ../../../Robots/Transform/Transform.cpp  -I../../../Robots/THOROP -I../../../Robots/Transform

  Author: Stephen McGill <smcgill3@seas.upenn.edu> 2014
*/

//#include <vector>
#include "mex.h"
#include "THOROPKinematics.h"

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
  // Check arguments
  if ((nrhs < 1)  || !mxIsDouble(prhs[0]))
    mexErrMsgTxt("Need double input angles.");

  double *q_ptr = (double *)mxGetData(prhs[0]);
//  int m = mxGetM(prhs[0]);
//  int n = mxGetN(prhs[0]);

	Transform t = THOROP_kinematics_forward_l_leg(q_ptr);
	mxArray *tr = mxCreateDoubleMatrix(4, 4, mxREAL);
	double *tr_ptr = mxGetPr(tr);
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			*tr_ptr++ = t(j, i);
		}
	}
	plhs[0] = tr;
	
}
