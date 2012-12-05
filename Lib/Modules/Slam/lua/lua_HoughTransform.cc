#ifdef __cplusplus
extern "C" {
#endif

#include "lua.h"
#include "lualib.h"
#include "lauxlib.h"

#ifdef __cplusplus
}
#endif

#include "HoughTransform.hh"
#include <iostream>
#include "Timer.hh"
#include <string.h>

// Arguments
enum {XS,YS,A_CENTER,A_RANGE,A_RES,R_CENTER,R_RANGE,R_RES};

using namespace std;

vector<double> angles;
vector<double> rhos;
int * h_transform = NULL;
int nh = 0;

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
  Upenn::Timer timer0;

	//TODO: Generate error for incorrect number of parameters
	/*
  if (nrhs != 8)
    mexErrMsgTxt("incorrect number of arguments");
		*/
	
  vector< pair<double, double> > points;
  double angle_center, angle_range, angle_res, rho_center, rho_range, rho_res;

  double * xs = mxGetPr(prhs[XS]);
  double * ys = mxGetPr(prhs[YS]);

  int n_xs = mxGetNumberOfElements(prhs[XS]);
  int n_ys = mxGetNumberOfElements(prhs[YS]);

  if (n_xs != n_ys)
    mexErrMsgTxt("xs and ys vectors must be have the same length");

  angle_center = mxGetPr(prhs[A_CENTER])[0];
  angle_range  = mxGetPr(prhs[A_RANGE])[0];
  angle_res    = mxGetPr(prhs[A_RES])[0];
  
  rho_center   = mxGetPr(prhs[R_CENTER])[0];
  rho_range    = mxGetPr(prhs[R_RANGE])[0];
  rho_res      = mxGetPr(prhs[R_RES])[0];

  int n_angles, n_rhos;

  //generate the angles
  GenerateRange(angle_center, angle_range, angle_res, angles, n_angles);

  //generate rhos
  GenerateRange(rho_center, rho_range, rho_res, rhos, n_rhos);

#ifdef HOUGH_TRANSFORM_API_DEBUG

  cout<<"number of points ="<<n_xs<<endl;
  
  cout<<"angles: ";
  for (int i=0; i<angles.size(); i++)
    cout<<angles[i]<<" ";
  cout<<endl;

  cout<<"rhos: ";
  for (int i=0; i<rhos.size(); i++)
    cout<<rhos[i]<<" ";
  cout<<endl;
#endif

  int n_out = n_angles*n_rhos;
  if (n_out > nh)
  {
    if (h_transform)
      delete [] h_transform;

    h_transform = new int[n_out];
    nh = n_out;
  }

  if (CalculateHoughTransform(xs, ys, n_xs, angles, rho_center, rho_range, rho_res, h_transform) != 0)
    mexErrMsgTxt("Could not compute Hough Transform");
  
  if (nlhs > 0)
  {    
    plhs[0] = mxCreateDoubleMatrix(n_rhos, n_angles, mxREAL);
    double * p_out = mxGetPr(plhs[0]);
    int * h_tr = h_transform;

    for (int i=0; i< n_out; i++)
      *(p_out++) = *(h_tr++);
  }

  if (nlhs>1)
  {
    plhs[1] = mxCreateDoubleMatrix(n_angles,1,mxREAL);
    memcpy(mxGetPr(plhs[1]),&(angles[0]),n_angles*sizeof(double));
  }

  if (nlhs>2)
  {
    plhs[2] = mxCreateDoubleMatrix(n_rhos,1,mxREAL);
    memcpy(mxGetPr(plhs[2]),&(rhos[0]),n_rhos*sizeof(double));
  }
}

