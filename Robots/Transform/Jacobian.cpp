#include "Transform.h"

Jacobian7::Jacobian7() {clear();}

void Jacobian7::clear() {
  // Initialize to identity matrix:
  for (int i = 0; i < 7; i++){
    inertia[i]=0;
    for (int j = 0; j < 3; j++){
      v[i][j]=0;
      w[i][j]=0;
    }
  }
}

Jacobian7& Jacobian7::calculate(
    const Transform &A, 
    const Transform &Adot0,
    const Transform &Adot1,
    const Transform &Adot2,
    const Transform &Adot3,
    const Transform &Adot4,
    const Transform &Adot5,
    const Transform &Adot6) {

  Adot0.getXYZ(v[0]);
  Adot1.getXYZ(v[1]);
  Adot2.getXYZ(v[2]);
  Adot3.getXYZ(v[3]);
  Adot4.getXYZ(v[4]);
  Adot5.getXYZ(v[5]);
  Adot6.getXYZ(v[6]);

/*
  Transform Ainv = inv(A);
  getAngularVelocityTensor(Adot0, Ainv, w[0]);
  getAngularVelocityTensor(Adot1, Ainv, w[1]);
  getAngularVelocityTensor(Adot2, Ainv, w[2]);
  getAngularVelocityTensor(Adot3, Ainv, w[3]);
  getAngularVelocityTensor(Adot4, Ainv, w[4]);
  getAngularVelocityTensor(Adot5, Ainv, w[5]);
  getAngularVelocityTensor(Adot6, Ainv, w[6]);
*/


  int i,j;
  for(i=0;i<7;i++){
    for(j=0;j<7;j++){
      inertia[i][j]=
        v[i][0]*v[j][0]+
        v[i][1]*v[j][1]+
        v[i][2]*v[j][2];
    }
  }

  return *this;
}

void Jacobian7::accumulate_torque(
  double* torque, 
  double forcex, double forcey, double forcez){
  for(int i=0;i<7;i++){
    torque[i]+=
      v[i][0]*forcex+
      v[i][1]*forcey+
      v[i][2]*forcez;
  }
}

void Jacobian7::accumulate_inertia(
  double* inertia_matrix){
  for(int i=0;i<7;i++)
    for(int j=0;j<7;i++)
      inertia_matrix[i*7+j]+=inertia[i][j];
}
