/* 
 * Lidar processing pack for matlab
 *
 * SJ 2013
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * */

#include <unistd.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <msgpack.h>

#include "mex.h"
#include "matrix.h"
#include "math.h"

#define LIDAR_HEIGHT 0.10
#define NECK_HEIGHT 0.30
#define CHEST_LIDAR_HEIGHT 0.10
#define CHEST_LIDAR_JOINT_X 0.05
#define CHEST_LIDAR_OFFSET_X 0.05

typedef unsigned char uint8;
typedef unsigned int uint32;



void get_head_projection(float* rayend, float rayAngle, float lidarAngle, float range){
  float ca = cos(lidarAngle);
  float sa = sin(lidarAngle);
  float dx = range*cos(rayAngle);
  float dy = range*sin(rayAngle);
  float dz = LIDAR_HEIGHT;
  rayend[0] = ca*dx + sa*dz;
  rayend[1] = dy;
  rayend[2] = -sa*dx + ca*dz + NECK_HEIGHT;
}

void get_chest_projection(float* rayend, float rayAngle, float lidarAngle, float range){
//printf("lidarangle %f chestangle %f range %f\n",lidarAngle,chestAngle,range);
  float ca = cos(lidarAngle);
  float sa = sin(lidarAngle);

  float dx = CHEST_LIDAR_OFFSET_X + range*cos(rayAngle);
  float dz = range*sin(-rayAngle);
  rayend[0] = ca*dx + CHEST_LIDAR_JOINT_X;
  rayend[1] = sa*dx;
  rayend[2] = dz+CHEST_LIDAR_HEIGHT;

}



mxArray* mex_get_head_projection(int nrhs, const mxArray *prhs[]){
  float lidarAngle = mxGetScalar(prhs[0]);
  float neckAngle = mxGetScalar(prhs[1]);
  float range = mxGetScalar(prhs[2]);

  float rayend[3];
  get_head_projection(rayend, lidarAngle, neckAngle, range);

  mxArray* ret = mxCreateDoubleMatrix(1,3, mxREAL);
  double * retPtr = mxGetPr(ret);
  retPtr[0]=rayend[0];
  retPtr[1]=rayend[1];
  retPtr[2]=rayend[2];
  
  return ret;
}

mxArray* mex_get_chest_projection(int nrhs, const mxArray *prhs[]){
  float rayAngle = mxGetScalar(prhs[0]);
  float lidarAngle = mxGetScalar(prhs[1]);
  float range = mxGetScalar(prhs[2]);
  float rayend[3];
  get_chest_projection(rayend, rayAngle, lidarAngle, range);

  mxArray* ret = mxCreateDoubleMatrix(1,3, mxREAL);

  double * retPtr = mxGetPr(ret);
  retPtr[0] = (double) rayend[0];
  retPtr[1] = (double) rayend[1];
  retPtr[2] = (double) rayend[2];
  return ret;
}

/*
void mex_smooth_map(mxArray* ret[], int nrhs, const mxArray *prhs[], int lidarType){
    double* range_ptr =  mxGetPr(prhs[0]);
    double connect_threshold = mxGetScalar(prhs[1]);

    int m_range = mxGetM(prhs[2]);
    int n_range = mxGetN(prhs[2]);

    for (j=1;j<n_range-1;j++)
      for (i=1;i<m_range-1;i++){
        int vert_index = i+j*m_range;
	int vert_index_t = (i-1)+ m_range * j);
	int vert_index_b = (i+1)+ m_range * j ;
	int vert_index_l = (i)+  m_range * (j-1) ;
	int vert_index_r = (i)+  m_range * (j+1)  ;

	double t = range_ptr(vert_index_t);
	double b = range_ptr(vert_index_b);
	double l = range_ptr(vert_index_l);
	double r = range_ptr(vert_index_r);

        float connect_threshold1 = connect_threshold/(dist1+1.0);
//TODO
   }
}
*/

void mex_get_map(mxArray* ret[], int nrhs, const mxArray *prhs[], int lidarType){
    int i,j;
    //ray angles
    double* rayAngle_ptr = mxGetPr(prhs[0]);
    int n_ray = mxGetN(prhs[0]);
    int m_ray = mxGetM(prhs[0]);

    //lidar angles (chest servo angles)
    double* lidarAngle_ptr = mxGetPr(prhs[1]);
    int m_lidar = mxGetM(prhs[1]);
    int n_lidar = mxGetN(prhs[1]);

    //Ranges
    double* range_ptr =  mxGetPr(prhs[2]);
    int m_range = mxGetM(prhs[2]);
    int n_range = mxGetN(prhs[2]);

    if ((m_range!=m_ray) || (n_range!=n_lidar))
      mexErrMsgTxt("Input dimension mismatch");

    printf("range:%d %d\n",m_range,n_range);

    mxArray* retX = mxCreateDoubleMatrix(m_range,n_range, mxREAL);
    mxArray* retY = mxCreateDoubleMatrix(m_range,n_range, mxREAL);
    mxArray* retZ = mxCreateDoubleMatrix(m_range,n_range, mxREAL);

    double* retX_ptr = mxGetPr(retX);
    double* retY_ptr = mxGetPr(retY);
    double* retZ_ptr = mxGetPr(retZ);

    for (j=0;j<n_range;j++)
      for (i=0;i<m_range;i++){

        float temp[3];
	if (lidarType>0){
          get_chest_projection(temp, rayAngle_ptr[i], lidarAngle_ptr[j],
		range_ptr[i+j*m_range]);
        }else{
          get_head_projection(temp, rayAngle_ptr[i], lidarAngle_ptr[j],
		range_ptr[i+j*m_range]);
        }

        retX_ptr[i+j*m_range] = temp[0];
        retY_ptr[i+j*m_range] = temp[1];
	retZ_ptr[i+j*m_range] = temp[2];
    }
    ret[0] = retX;
    ret[1] = retY;
    ret[2] = retZ;
}

void smooth_mesh(double* ret_ptr, double* range_ptr, 
	double* rayAngle_ptr, double* lidarAngle_ptr, 
	int m_range, int n_range, float connect_threshold){

  int i,j;
  for (i=1;i<m_range-1;i++)
    for (j=1;j<n_range-1;j++){
      int vert_index = i + m_range * j;
      int vert_index_t = (i-1)+ m_range * j;
	int vert_index_b = (i+1)+ m_range * j;
	int vert_index_l = (i)+  m_range * (j-1);
	int vert_index_r = (i)+  m_range * (j+2);

        float dist = range_ptr[vert_index];
        float dist_t = range_ptr[vert_index_t];
        float dist_b = range_ptr[vert_index_b];
        float dist_l = range_ptr[vert_index_l];
        float dist_r = range_ptr[vert_index_r];

        float ray_dist = (rayAngle_ptr[i+1]-rayAngle_ptr[i-1])*
		 range_ptr[vert_index];
        float lidar_dist = (lidarAngle_ptr[j+1]-lidarAngle_ptr[j-1])*
		 range_ptr[vert_index];

	float dist_err_lr = fabs(dist_l-dist_r);
	float dist_err_tb = fabs(dist_t-dist_b);

	//connect threshold: tangent of the angle
        int mesh_connected = 1;

        if (dist_err_lr > lidar_dist * connect_threshold) mesh_connected = 0;
        if (dist_err_tb > ray_dist * connect_threshold) mesh_connected = 0;
//        if ((dist<0.10)||(dist>cut_threshold))  mesh_connected = 0;
	float average_dist = (dist_t+dist_b+dist_l+dist_r)/4;
	float gamma = 0.5;
	if (mesh_connected>0)
	  ret_ptr[vert_index] = gamma*average_dist + (1-gamma)*dist;
	else ret_ptr[vert_index]=range_ptr[vert_index];
   }
}




void mex_get_mesh(mxArray* ret[], int nrhs, const mxArray *prhs[], int lidartype){
    int i,j;
    double* rayAngle_ptr = mxGetPr(prhs[0]);
    int n_ray = mxGetN(prhs[0]);
    int m_ray = mxGetM(prhs[0]);

    //lidar pan angles 
    double* lidarAngle_ptr = mxGetPr(prhs[1]);
    int m_lidar = mxGetM(prhs[1]);
    int n_lidar = mxGetN(prhs[1]);

    //Ranges
    double* range_ptr =  mxGetPr(prhs[2]);
    int m_range = mxGetM(prhs[2]);
    int n_range = mxGetN(prhs[2]);

    double connect_threshold = mxGetScalar(prhs[3]);
    double cut_threshold = mxGetScalar(prhs[4]);
    double ground_height = mxGetScalar(prhs[5]);
    double max_height = mxGetScalar(prhs[6]);

    if ((m_range!=m_ray) || (n_range!=n_lidar))
      mexErrMsgTxt("Input dimension mismatch");

    printf("range:%d %d\n",m_range,n_range);

    int max_face_no = (m_range-1)*(n_range-1)*2;
    int max_vert_no = m_range*n_range;

    mxArray* face = mxCreateDoubleMatrix(3,max_face_no, mxREAL);
    mxArray* vert = mxCreateDoubleMatrix(3,max_vert_no, mxREAL);
    mxArray* cdata = mxCreateDoubleMatrix(3,max_face_no, mxREAL);

    double* face_ptr = mxGetPr(face);
    double* vert_ptr = mxGetPr(vert);
    double* cdata_ptr = mxGetPr(cdata);

    //Initialize vertex matrix
    for (i=0;i<m_range;i++)
      for (j=0;j<n_range;j++){
        int vert_index = i+j*m_range;
        float temp[3];
       	if (lidartype==0)
          get_head_projection(temp, rayAngle_ptr[i], lidarAngle_ptr[j],	range_ptr[vert_index]);
        else
          get_chest_projection(temp, rayAngle_ptr[i], lidarAngle_ptr[j],		range_ptr[vert_index]);
        //Set vertex xyz positions
        vert_ptr[vert_index*3] = temp[0];
        vert_ptr[vert_index*3+1] = temp[1];
        vert_ptr[vert_index*3+2] = temp[2];
    }

    //Add mesh matrix
    int face_count = 0;
    for (i=1;i<m_range;i++)
      for (j=1;j<n_range;j++){
        int mesh_connected1 = 1;
        int mesh_connected2 = 1;

				int vert_index_tl = (i-1)+ m_range * (j-1);
				int vert_index_tr = (i-1)+ m_range * (j) ;
				int vert_index_bl = (i)+  m_range * (j-1) ;
				int vert_index_br = (i)+  m_range * (j)  ;

        float dist_tl = range_ptr[vert_index_tl];
        float dist_tr = range_ptr[vert_index_tr];
        float dist_bl = range_ptr[vert_index_bl];
        float dist_br = range_ptr[vert_index_br];

        float ray_dist = (rayAngle_ptr[i]-rayAngle_ptr[i-1]) * range_ptr[vert_index_tl];
        float lidar_dist = (lidarAngle_ptr[j]-lidarAngle_ptr[j-1])* range_ptr[vert_index_tl];

				float dist_err1 = fabs(dist_tl-dist_tr);
				float dist_err2 = fabs(dist_tl-dist_bl);
				float dist_err3 = fabs(dist_tr-dist_br);
				float dist_err4 = fabs(dist_bl-dist_br);

			

	//connect threshold: tangent of the angle

	float dist_th = 0.05;
	float dist_min = 0.10;

	//Check top left triangle
        if ((dist_err1>dist_th)&&(dist_err1>lidar_dist * connect_threshold)) mesh_connected1 = 0;
        if ((dist_err2>dist_th)&&(dist_err2>ray_dist * connect_threshold)) mesh_connected1 = 0;
        if ((dist_tl<dist_min)||(dist_tl>cut_threshold))  mesh_connected1 = 0;

	//Check bottom right triangle
        if ((dist_err4>dist_th)&&(dist_err4 > lidar_dist * connect_threshold)) mesh_connected2 = 0;
        if ((dist_err3>dist_th)&&(dist_err3 > ray_dist * connect_threshold)) mesh_connected2 = 0;
        if ((dist_br<dist_min)||(dist_br>cut_threshold))  mesh_connected2 = 0;

        float z_height1= vert_ptr[vert_index_tl*3+2];
        float z_height2= vert_ptr[vert_index_br*3+2];

			  if (z_height1>max_height) mesh_connected1=0;
			  if (z_height2>max_height) mesh_connected2=0;

	//Matlab index start with 1
        if (mesh_connected1>0) {
	    face_ptr[face_count*3] = vert_index_tl+1;
	    face_ptr[face_count*3+1] = vert_index_tr+1;
  	    face_ptr[face_count*3+2] = vert_index_bl+1;
            if (z_height1>ground_height) {
              cdata_ptr[face_count*3+0] = 0.9;
              cdata_ptr[face_count*3+1] = 0.3;
              cdata_ptr[face_count*3+2] = 0.3;
            }else{
              cdata_ptr[face_count*3+0] = 0.5;
              cdata_ptr[face_count*3+1] = 0.5;
              cdata_ptr[face_count*3+2] = 0.5;
	    }
	    face_count = face_count + 1;
	}
        if (mesh_connected2>0) {
	    face_ptr[face_count*3] = vert_index_tr+1;
	    face_ptr[face_count*3+1] = vert_index_br+1;
  	    face_ptr[face_count*3+2] = vert_index_bl+1;
            if (z_height2>ground_height) {
              cdata_ptr[face_count*3+0] = 0.9;
              cdata_ptr[face_count*3+1] = 0.3;
              cdata_ptr[face_count*3+2] = 0.3;
            }else{
              cdata_ptr[face_count*3+0] = 0.5;
              cdata_ptr[face_count*3+1] = 0.5;
              cdata_ptr[face_count*3+2] = 0.5;
	    }
	    face_count = face_count + 1;
	}
    }

    mxArray* m_face_count = mxCreateDoubleMatrix(1,1, mxREAL);
    double* face_count_ptr = mxGetPr(m_face_count);
    face_count_ptr[0]=face_count;


    ret[0] = vert;
    ret[1] = face;
    ret[2] = cdata;
    ret[3] = m_face_count;
}












//usage: 'chestproject', lidarangle, chestangle, range


void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
  if ((nrhs < 1) || (!mxIsChar(prhs[0])))
    mexErrMsgTxt("Need to input string argument");

  char *fname = mxArrayToString(prhs[0]);
  if (strcmp(fname, "headproject") == 0) {
    plhs[0] = mex_get_head_projection(nrhs-1,prhs+1);
  }else if (strcmp(fname, "chestproject") == 0) {
    plhs[0] = mex_get_chest_projection(nrhs-1,prhs+1);
  }else if (strcmp(fname, "chestmap") == 0) {
    mex_get_map(plhs,nrhs-1,prhs+1,1);
  }else if (strcmp(fname, "headmap") == 0) {
    mex_get_map(plhs,nrhs-1,prhs+1,0);
  }else if (strcmp(fname, "chestmesh") == 0) {
    mex_get_mesh(plhs,nrhs-1,prhs+1, 1);
  }else if (strcmp(fname, "headmesh") == 0) {
    mex_get_mesh(plhs,nrhs-1,prhs+1, 0);
  } else 
    mexErrMsgTxt("Unknown function argument");

}

