#include "fitPlane.h"
#include <iostream>
#include <cmath>

void CFitPlane::SetNeighborSize(int nw)
{
    if (nw != mnW && nw > 0){
        mnW = nw;
        mnK = 2*nw+1;
    }    
        
    return;
}

void CFitPlane::SetSubsampleStep(int step)
{
    if (step > 0)
        subsampleStep = step;
    
    return;
}

bool CFitPlane::Init(double *u, double *v, double *d,bool* mask, int ih, int iw)
{
    return Init(u,v,d,mask,ih,iw,2);
}

bool CFitPlane::Init(double *u, double *v, double *d,bool* mask, int ih, int iw, int nw)
{
    if (nw != mnW && nw > 0){
        mnW = nw;
        mnK = 2*nw+1;
    }    
    else
        return false;
    
    if (subsampleStep < 1)
            subsampleStep = mnW;
           
    if (iw > 0 && ih > 0){
        
        w = iw;
        h = ih;
        sz = w*h;
          
        vec3D.setZero(3,sz);
        vecCoeffs.setZero(4,sz);
        vecSValues.setZero(4,sz);        
        matValid.setZero(sz,1);
        matMask.setZero(sz,1);
        
       int test = 0;
        for (int i=0;i<sz;i++){     
            if ( isnan(*(d+i)) || *(mask+i) == false) {                
                vec3D(0,i) = 0;
                vec3D(1,i) = 0;
                vec3D(2,i) = 0;
                matMask(i) == 0;
            }
            else {
                vec3D(0,i) = *(u+i);
                vec3D(1,i) = *(v+i);
                vec3D(2,i) = *(d+i);              
                matMask(i) = 1;   
            }
        }
        return true;
    }
    else
        return false;
}

void CFitPlane::fitPlane()
{         
    int nK2 = mnK*mnK; 
    int test = 0;
              
    // Find neighbor within a box patch around each point       
    for (int i=mnW;i<(w-mnW);i+=subsampleStep){ 
         for (int j=mnW;j<(h-mnW);j+=subsampleStep){

             int cind = i*h+j;             
             
             bool valid = true;
             Eigen::MatrixXd P = Eigen::MatrixXd::Zero(4,nK2);
             for (int k=0;k<mnK;k++){
                 for ( int l=0;l<mnK;l++){
                     int ind = (i+k-mnW)*h+(j+l-mnW);
                     int pind = k*mnK+l;           
                     
                     //std::cout << "( " << i+k-nKw <<", " << j+l-nKw << ")" << std::endl;                                        
                     if (matMask(ind) == 0){                    
                          valid = false;
                          break;
                     }
                     
                     P(0,pind) = vec3D(0,ind);
                     P(1,pind) = vec3D(1,ind);
                     P(2,pind) = 1.0f;
                     P(3,pind) = vec3D(2,ind);
                 }
                     
                 if (valid != true) break;
             }                                 
             
             if (valid == true ){
                 
                 double norm = 1.0;
                 // compute normals and singular values
                 Eigen::JacobiSVD<Eigen::MatrixXd> svd(P, Eigen::ComputeThinU );
                 
                // if (test == 0){
               //  std::cout << P << std::endl; 
               //  test = 1;
              //   }
                         
                 Eigen::MatrixXd U =  svd.matrixU();
                 vecSValues.col(cind) = svd.singularValues();              
                 vecCoeffs.col(cind) = U.col(3);   
                 // normalize
                 norm = sqrt(vecCoeffs(0,cind)*vecCoeffs(0,cind) + vecCoeffs(1,cind)*vecCoeffs(1,cind) + vecCoeffs(2,cind)*vecCoeffs(2,cind));
                 vecCoeffs.col(cind) = vecCoeffs.col(cind)/norm;
                 matValid(cind) = 1;
            }
         }
    }
     
    return;
}
    
Eigen::MatrixXd CFitPlane::GetCoeffs() // k: number of neighbor
{    
    return vecCoeffs;
}

Eigen::MatrixXd CFitPlane::GetSingValues() 
{    
    return vecSValues;
}

Eigen::VectorXi CFitPlane::GetValidity() 
{    
    return matValid;
}

