#include "joint_statics.h"
#include "joint_kinematics.h"
#include "Transform.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

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
  double ujoint_bot[3];
  double ujoint_top[3];

  memcpy(ujoint_bot, actuator_bot[act_index], 3*sizeof(double));
  memcpy(ujoint_top, actuator_top[act_index], 3*sizeof(double));

  t.apply(ujoint_bot);
  act=subtract_vectors(&ujoint_top[0], &ujoint_bot[0]);
  act=scalar_mult(1/r,&act[0]);
  lever=subtract_vectors(&links[link_index][0],&ujoint_bot[0]);
  torque=cross_product(&lever[0],&act[0]);  
  torque=scalar_mult(v,&torque[0]);
  
  return torque;
}

std::vector<double>
statics_forward_joints(const double *v, const double *q, const double *r)
{
  /* forward statics to convert servo forces to joint torques */
  std::vector<double> t(1);
  std::vector<double> s(1);
  std::vector<double> torque(3);
  Transform t_joint;
  double joint_axes[3];

  memcpy(joint_axes, axes[0], 3*sizeof(double));

  // translate servo position to match zero pose coordinates
  s[0] = r[0] + home_offset[0];

  // left knee actuator
  t_joint.clear();
  t_joint.translate(links[0][0], links[0][1], links[0][2]).rotateY(q[0]);
  torque = find_unit_torque(t_joint, 0, 1, s[0], v[0]);
  t[0] = dot_product(&torque[0], joint_axes);
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
  std::vector<double> v(1);
  std::vector<double> s(1);
  std::vector<double> torque_1(3);
  Transform t_joint;
  double joint_axes[3];
  double fot[2][2]={{0,0},{0,0}};
  
  memcpy(joint_axes, axes[0], 3*sizeof(double));

  // translate servo positions to match zero pose coordinates
  s[0] = r[0] + home_offset[0];
  
  // left knee
  t_joint.clear();
  t_joint.translate(links[0][0], links[0][1], links[0][2]).rotateY(q[0]);
  torque_1 = find_unit_torque(t_joint, 0, 1, s[0], 1);
  fot[0][0] = 1/dot_product(joint_axes, &torque_1[0]);
  v[0] = t[0]*fot[0][0];
  
  return v;
}
