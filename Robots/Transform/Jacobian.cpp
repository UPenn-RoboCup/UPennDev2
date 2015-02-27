#include "Transform.h"

Jacobian7::Jacobian7() {clear();}

void Jacobian7::clear() {
  // Initialize to identity matrix:
  for (int i = 0; i < 7; i++)
    for (int j = 0; j < 3; j++){
      v[i][j]=0;
      w[i][j]=0;
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

//  Adot0.getXYZ(v[0]);
//  Adot1.getXYZ(v[1]);

//  Transform Ainv = inv(A);
//  getAngularVelocityTensor(Adot0, Ainv, w[0]);
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
