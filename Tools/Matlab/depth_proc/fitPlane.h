#ifndef FIT_PLANE_H
#define FIT_PLANE_H

#include <Eigen/Dense>

class CFitPlane {
    
public:
    // Input data is point cloud arranged in 1D array
    // i.e., X[n*height+m] gives X-value of (m,n) pixel
    CFitPlane(){ subsampleStep = 0;};    
    ~CFitPlane(){};
    
    bool Init(double *u, double *v, double *d,bool* mask, int ih, int iw, int K);
    bool Init(double *u, double *v, double *d,bool* mask, int ih, int iw);
    void SetNeighborSize(int nw);    
    void SetSubsampleStep(int step);
    void fitPlane();
    
    Eigen::MatrixXd GetCoeffs(); 
    Eigen::MatrixXd GetSingValues();
    Eigen::VectorXi GetValidity();
     
private:
    
    int                 mnW;  // size of neighbor width/2
    int                 mnK; // kernel width = mnW*2+1
    int                 w,h,sz;
    int                 subsampleStep;
    
    Eigen::MatrixXd     vec3D;
    Eigen::VectorXi     matMask;
    Eigen::VectorXi     matValid;
    Eigen::MatrixXd     vecCoeffs;
    Eigen::MatrixXd     vecSValues;
    
};
#endif