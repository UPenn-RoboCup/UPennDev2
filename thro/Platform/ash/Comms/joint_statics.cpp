#include "joint_statics.h"
#include "joint_kinematics.h"
#include "Transform.h"
#include <math.h>
#include <stdio.h>

std::vector<double> subtract_vectors(const double *v1, const double *v2)
{
  std::vector<double> t(3);
  for (int i=0; i<3; i++){
    t[i]=v1[i]-v2[i];
  }
  return t;
}

std::vector<double> add_vectors(const double *v, const double *w)
{
  std::vector<double> t(3);
  for (int i=0; i<3; i++){
    t[i]=v[i]+w[i];
  }
  return t;
}

std::vector<double> scalar_mult( double s, const double *v)
{
  std::vector<double> t(3);
  for (int i=0; i<3; i++){
    t[i]=s*v[i];
  }
  return t;                   
}

std::vector <double> cross_product(const double *v, const double *w)
{
  std::vector <double> t(3);
  t[0]=v[1]*w[2]-v[2]*w[1];
  t[1]=v[2]*w[0]-v[0]*w[2];
  t[2]=v[0]*w[1]-v[1]*w[0];
  return t;
}

double dot_product(const double *v, const double *w)
{
  double dp;
  dp=0;
  dp=v[0]*w[0]+v[1]*w[1]+v[2]*w[2];
  return dp;     
}

std::vector<double> find_unit_torque(Transform t, int act_index, int link_index, double r, double v)
{
  // t is the joint transform, r is the actuator length, v is the force
  std::vector<double> torque(3);
  std::vector<double> act(3);
  std::vector<double> lever(3);
  double actuator_bot[3];

  for (int i = 0; i < 3; i++) {
    actuator_bot[i]=actuatorBot[act_index][i];
  }
  t.apply(actuator_bot);
  act=subtract_vectors(&actuatorTop[act_index][0], &actuator_bot[0]);
  act=scalar_mult(1/r,&act[0]);
  lever=subtract_vectors(&links[link_index][0],&actuator_bot[0]);
  torque=cross_product(&lever[0],&act[0]);  
  torque=scalar_mult(v,&torque[0]);
  
  return torque;
}

std::vector<double>
statics_forward_joints(const double *v, const double *q, const double *r)
{
  /* forward statics to convert servo forces to joint torques */
  std::vector<double> t(12);
  std::vector<double> act_len(12);
  std::vector<double> torque(3);
  std::vector<double> torque_1(3);
  std::vector<double> torque_2(3);
  Transform t_joint;
  Transform t_axes;
  double Axes [12][3];
  
  for (int i=0; i<12; i++){
    for (int i2=0; i2<3; i2++){
        Axes[i][i2]=axes[i][i2];
    }
  }
  // translate servo positions to match zero pose coordinates
  for (int i = 0; i < 12; i++) {
    act_len[i] = r[i] + servoOffset[i];
  }

  //left hip actuators
  t_joint.translateY(links[0][1]).rotateZ(q[0]).rotateX(q[1]).rotateY(q[2]);  
  t_axes.rotateZ(q[0]).apply(Axes[1]);
  t_axes.rotateX(q[1]).apply(Axes[2]);
  torque_1=find_unit_torque(t_joint, 1, 0, act_len[1], v[1]);
  torque_2=find_unit_torque(t_joint, 2, 0, act_len[2], v[2]);
  torque=add_vectors(&torque_1[0], &torque_2[0]);
  t[0]=torque_1[2]+torque_2[2]+v[0];
  t[1]=dot_product(&torque[0], Axes[1]);
  t[2]=dot_product(&torque[0], Axes[2]);
 
  //left knee actuator
  t_joint.clear();
  t_joint.translate(links[1][0],0,links[1][2]).rotateY(q[3]); 
  torque=find_unit_torque(t_joint, 3, 1, act_len[3], v[3]);
  t[3]=dot_product(&torque[0], Axes[3]);
  
  //left ankle actuators
  t_joint.clear(); 
  t_axes.clear();
  t_joint.translate(links[2][0],0,links[2][2]).rotateY(q[4]).rotateX(q[5]);  
  t_axes.rotateY(q[4]).apply(Axes[5]);
  torque_1=find_unit_torque(t_joint, 4, 2, act_len[4], v[4]);
  torque_2=find_unit_torque(t_joint, 5, 2, act_len[5], v[5]);
  torque=add_vectors(&torque_1[0], &torque_2[0]);
  t[4]=dot_product(&torque[0], Axes[4]);
  t[5]=dot_product(&torque[0], Axes[5]);  

  //right hip actuators
  t_joint.clear(); 
  t_axes.clear();
  t_joint.translateY(links[3][1]).rotateZ(q[6]).rotateX(q[7]).rotateY(q[8]);  
  t_axes.rotateZ(q[6]).apply(Axes[7]);
  t_axes.rotateX(q[7]).apply(Axes[8]);
  torque_1=find_unit_torque(t_joint, 7, 3, act_len[7], v[7]);
  torque_2=find_unit_torque(t_joint, 8, 3, act_len[8], v[8]);
  torque=add_vectors(&torque_1[0], &torque_2[0]);
  t[6]=torque_1[2]+torque_2[2]+v[6];
  t[7]=dot_product(&torque[0], Axes[7]);
  t[8]=dot_product(&torque[0], Axes[8]);
 
  //right knee actuator
  t_joint.clear();
  t_joint.translate(links[4][0],0,links[4][2]).rotateY(q[9]); 
  torque=find_unit_torque(t_joint, 9, 4, act_len[9], v[9]);
  t[9]=dot_product(&torque[0], Axes[9]);
  
  //right ankle actuators
  t_joint.clear(); 
  t_axes.clear();
  t_joint.translate(links[5][0],0,links[5][2]).rotateY(q[10]).rotateX(q[11]);  
  t_axes.rotateY(q[10]).apply(Axes[11]);
  torque_1=find_unit_torque(t_joint, 10, 5, act_len[10], v[10]);
  torque_2=find_unit_torque(t_joint, 11, 5, act_len[11], v[11]);
  torque=add_vectors(&torque_1[0], &torque_2[0]);
  t[10]=dot_product(&torque[0], Axes[10]);
  t[11]=dot_product(&torque[0], Axes[11]);    
  return t; 
}

void matrix_inverse(double invert[][2])
{
  double matrix[2][2];
  double determinant;
  for (int i=0; i<2; i++){
    for (int i2=0; i2<2; i2++){
      matrix[i][i2]=invert[i][i2];
    }
  }
  determinant=matrix[0][0]*matrix[1][1]-matrix[0][1]*matrix[1][0];
  invert[0][0]=matrix[1][1]/determinant;
  invert[0][1]=-1*matrix[0][1]/determinant;
  invert[1][0]=-1*matrix[1][0]/determinant;
  invert[1][1]=matrix[0][0]/determinant;
}

void generate_FOT(double fot[][2], const double *t1, const double *t2,
                                   double axes1[], double axes2[])
{
  fot[0][0]=dot_product(axes1, &t1[0]);
  fot[0][1]=dot_product(axes1, &t2[0]);
  fot[1][0]=dot_product(axes2, &t1[0]);
  fot[1][1]=dot_product(axes2, &t2[0]);
  matrix_inverse(fot);
}

std::vector<double>
statics_inverse_joints(const double *t, const double *q, const double *r)
{
  /* inverse statics to convert joint torques to servo forces */
  std::vector<double> v(12);
  std::vector<double> act_len(12);
  std::vector<double> torque_1(3);
  std::vector<double> torque_2(3);
  Transform t_joint;
  Transform t_axes;
  Transform t_axes_2;
  double Axes [12][3];
  double fot[2][2]={{0,0},{0,0}};
  
  for (int i=0; i<12; i++){
    for (int i2=0; i2<3; i2++){
      Axes[i][i2]=axes[i][i2];
    }
  }
  // translate servo positions to match zero pose coordinates
  for (int i = 0; i < 12; i++) {
    act_len[i] = r[i] + servoOffset[i];
  }
  
  // left hip
  t_joint.translateY(links[0][1]).rotateZ(q[0]).rotateX(q[1]).rotateY(q[2]);  
  t_axes.rotateZ(q[0]).apply(Axes[1]);
  t_axes.rotateX(q[1]).apply(Axes[2]); 
  torque_1=find_unit_torque(t_joint, 1, 0, act_len[1], 1);
  torque_2=find_unit_torque(t_joint, 2, 0, act_len[2], 1);
  generate_FOT(fot, &torque_1[0], &torque_2[0], Axes[1], Axes[2]);
  v[0]=t[0]-torque_1[2]-torque_2[2];
  v[1]=fot[0][0]*t[1]+fot[0][1]*t[2];
  v[2]=fot[1][0]*t[1]+fot[1][1]*t[2];
  
  // left knee
  t_joint.clear();
  t_joint.translateX(links[1][0]).translateZ(links[1][2]).rotateY(q[3]);  
  torque_1=find_unit_torque(t_joint, 3, 1, act_len[3], 1);
  fot[0][0]=1/dot_product(Axes[3], &torque_1[0]);
  v[3]=t[3]*fot[0][0];
  
  // left ankle
  t_joint.clear();
  t_axes.clear();
  t_joint.translateX(links[2][0]).translateZ(links[2][2]).rotateY(q[4]).rotateX(q[5]);  
  t_axes.rotateY(q[4]).apply(Axes[4]);
  t_axes.rotateX(q[5]).apply(Axes[5]);  
  torque_1=find_unit_torque(t_joint, 4, 2, act_len[4], 1);
  torque_2=find_unit_torque(t_joint, 5, 2, act_len[5], 1);
  generate_FOT(fot,&torque_1[0],&torque_2[0],Axes[4],Axes[5]);
  v[4]=fot[0][0]*t[4]+fot[0][1]*t[5];
  v[5]=fot[1][0]*t[4]+fot[1][1]*t[5];
  
  // right hip
  t_joint.clear();
  t_axes.clear();
  t_joint.translateY(links[3][1]).rotateZ(q[6]).rotateX(q[7]).rotateY(q[8]);  
  t_axes.rotateZ(q[6]).apply(Axes[7]);
  t_axes.rotateX(q[7]).apply(Axes[8]); 
  torque_1=find_unit_torque(t_joint, 7, 3, act_len[7], 1);
  torque_2=find_unit_torque(t_joint, 8, 3, act_len[8], 1);
  generate_FOT(fot, &torque_1[0], &torque_2[0], Axes[7], Axes[8]);
  v[6]=t[6]-torque_1[2]-torque_2[2];
  v[7]=fot[0][0]*t[7]+fot[0][1]*t[8];
  v[8]=fot[1][0]*t[7]+fot[1][1]*t[8];
  
  // right knee
  t_joint.clear();
  t_joint.translateX(links[4][0]).translateZ(links[4][2]).rotateY(q[9]);  
  torque_1=find_unit_torque(t_joint, 9, 4, act_len[9], 1);
  fot[0][0]=1/dot_product(Axes[9], &torque_1[0]);
  v[9]=t[9]*fot[0][0];
  
  // right ankle
  t_joint.clear();
  t_axes.clear();
  t_joint.translateX(links[5][0]).translateZ(links[5][2]).rotateY(q[10]).rotateX(q[11]);  
  t_axes.rotateY(q[10]).apply(Axes[10]);
  t_axes.rotateX(q[11]).apply(Axes[11]);  
  torque_1=find_unit_torque(t_joint, 10, 5, act_len[10], 1);
  torque_2=find_unit_torque(t_joint, 11, 5, act_len[11], 1);
  generate_FOT(fot,&torque_1[0],&torque_2[0],Axes[10],Axes[11]);
  v[10]=fot[0][0]*t[10]+fot[0][1]*t[11];
  v[11]=fot[1][0]*t[10]+fot[1][1]*t[11];
  
 return v;
}
