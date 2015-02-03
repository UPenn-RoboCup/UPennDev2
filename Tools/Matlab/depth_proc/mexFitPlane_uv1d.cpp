#include "mex.h"
#include <iostream>

#include "fitPlane.h"
#include <Eigen/Dense>

using namespace std;

void mexFunction (int nlhs,mxArray *plhs[],int nrhs,const mxArray *prhs[]) {

    // check arguments
    if (nrhs < 5) mexErrMsgTxt("At least 5 input arguments required.");

    // input pointers
    double *X        =    (double*)mxGetPr(prhs[0]);
    double *Y        =    (double*)mxGetPr(prhs[1]);
    double *Z        =    (double*)mxGetPr(prhs[2]);
    bool   *MASK     =      (bool*)mxGetPr(prhs[3]);
    double *param    =    (double*)mxGetPr(prhs[4]); 

    int dim1 = mxGetM(prhs[1]);       // (height) 
    int dim2 = mxGetN(prhs[1]);       // (width)
    int sz = dim1*dim2;

    // Output pointers
    if (dim1 < 1)   return;

    // Compute Geometry 
    CFitPlane fP;
    fP.Init(X,Y,Z,MASK, dim1,dim2);
    fP.SetNeighborSize((int)param[0]);    
    fP.SetSubsampleStep((int)param[1]);
    fP.fitPlane();
    Eigen::MatrixXd C = fP.GetCoeffs();  

    if (nlhs == 1 ){
        plhs[0]         = mxCreateDoubleMatrix(4,sz,mxREAL);
        double *pOut1   = (double*)mxGetPr(plhs[0]); 

        for (int k=0;k<dim1*dim2;k++)
        {
          int _4k = 4*k;
          *(pOut1+_4k+0) = C(0,k);
          *(pOut1+_4k+1) = C(1,k);
          *(pOut1+_4k+2) = C(2,k);          
          *(pOut1+_4k+3) = C(3,k);          
        }
    } else if (nlhs == 2 ){

        plhs[0]         = mxCreateDoubleMatrix(4,sz,mxREAL);
        double *pOut1   = (double*)mxGetPr(plhs[0]); 

        plhs[1]         = mxCreateDoubleMatrix(4,sz,mxREAL);
        double *pOut2   = (double*)mxGetPr(plhs[1]); 

        Eigen::MatrixXd S = fP.GetSingValues();  
        for (int k=0;k<dim1*dim2;k++)
        {
          int _4k = 4*k;
          *(pOut1+_4k+0) = C(0,k);
          *(pOut1+_4k+1) = C(1,k);
          *(pOut1+_4k+2) = C(2,k);
          *(pOut1+_4k+3) = C(3,k);          
          

          *(pOut2+_4k+0) = S(0,k);
          *(pOut2+_4k+1) = S(1,k);
          *(pOut2+_4k+2) = S(2,k);
          *(pOut2+_4k+3) = S(3,k);
        }
    } else if (nlhs == 3)
    {
        plhs[0]         = mxCreateDoubleMatrix(4,sz,mxREAL);
        double *pOut1   = (double*)mxGetPr(plhs[0]); 

        plhs[1]         = mxCreateDoubleMatrix(4,sz,mxREAL);
        double *pOut2   = (double*)mxGetPr(plhs[1]); 

        plhs[2]         = mxCreateDoubleMatrix(1,sz,mxREAL);
        double *pOut3   = (double*)mxGetPr(plhs[2]); 
        
        Eigen::MatrixXd S = fP.GetSingValues();  
        Eigen::VectorXi V = fP.GetValidity(); 
        for (int k=0;k<dim1*dim2;k++)
        {
          int _4k = 4*k;
          *(pOut1+_4k+0) = C(0,k);
          *(pOut1+_4k+1) = C(1,k);
          *(pOut1+_4k+2) = C(2,k);
          *(pOut1+_4k+3) = C(3,k);                  

          *(pOut2+_4k+0) = S(0,k);
          *(pOut2+_4k+1) = S(1,k);
          *(pOut2+_4k+2) = S(2,k);
          *(pOut2+_4k+3) = S(3,k);
          
          *(pOut3+k) = (double)V(k);
        }
        
    }


    return;
}
