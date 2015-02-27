#ifndef Transform_h_DEFINED
#define Transform_h_DEFINED

#include <math.h>
#include <stdio.h>
#include <vector>

class Transform {
public:
  Transform();
  virtual ~Transform() {}

  void clear();
  Transform &translate(double x, double y, double z);
  Transform &translate(const double *p);
  Transform &translateX(double x = 0);
  Transform &translateY(double y = 0);
  Transform &translateZ(double z = 0);
  Transform &rotateX(double a = 0);
  Transform &rotateY(double a = 0);
  Transform &rotateZ(double a = 0);
  Transform &rotateDotX(double a = 0);
  Transform &rotateDotY(double a = 0);
  Transform &rotateDotZ(double a = 0);

  Transform &mDH(double alpha, double a, double theta, double d);
  void apply(double x[3]);
  void apply0(double* x);

  
  double getZ();
  const void getXYZ(double* ret) const;
  
  
  double& operator() (int i, int j);
  const double operator() (int i, int j) const;

 private:
  double t[4][4];
};

Transform operator* (const Transform &t1, const Transform &t2);
Transform inv (const Transform &t1);
Transform trcopy (const Transform &t1);
Transform transform6D(const double p[6]);
std::vector<double> position6D(const Transform &t1);

void getAngularVelocityTensor(const Transform &adot, const Transform &ainv, double *av);


void printTransform(Transform tr);
void printVector(std::vector<double> v);

class Jacobian7 {
public:
  Jacobian7();
  virtual ~Jacobian7() {}

  Jacobian7 &calculate(
    const Transform &A, 
    const Transform &Adot0,
    const Transform &Adot1,
    const Transform &Adot2,
    const Transform &Adot3,
    const Transform &Adot4,
    const Transform &Adot5,
    const Transform &Adot6); 

  void clear();
  void accumulate_torque(double* torque,double forcex, double forcey, double forcez);
  void accumulate_inertia(double* inertia_matrix);
private:
  double v[7][3];
  double w[7][3];
  double inertia[7][7]; 
};






#endif
