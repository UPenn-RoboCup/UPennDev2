#include "YouBotKinematics.h"

void printTransform(Transform tr) {
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      printf("%.4g ", tr(i,j));
    }
    printf("\n");
  }
  printf("\n");
}

void printVector(std::vector<double> v) {
  for (int i = 0; i < v.size(); i++) {
    printf("%.4g\n", v[i]);
  }
  printf("\n");
}

//DH transform params: (alpha, a, theta, d)
Transform YouBot_kinematics_forward_arm(const double *q) {
  Transform t;
  t = t.translateZ(baseLength)
    .mDH(0, 0, q[0], 0)
    .mDH(-PI/2, 0, -PI/2+q[1], 0)
    .rotateX(PI/2).rotateY(PI/2);
  return t;
}

std::vector<double> YouBot_kinematics_inverse_arm(Transform trArm) {
double z = trArm(0,3);
double x = trArm(1,3);
double y = trArm(2,3);
  
  double x1 = sqrt(x*x + z*z);
  double y1 = y + wristLength + handLength - baseLength;
  
  double a = lowerArmLength;
  double b = upperArmLength;
  double c = sqrt(x1*x1 + y1*y1);
  
  double alpha = - asin( z / x1 );
  double beta = -(M_PI_2 - acos( (a*a + c*c - b*b) / (2.0*a*c) ) - atan( y1/x1 ));
  double gamma = -(M_PI - acos( (a*a + b*b - c*c) / (2.0*a*b) ));
  double delta = -(M_PI + (beta + gamma));
  double epsilon = M_PI_2 + alpha;
  
  std::vector<double> qArm(7);
  qArm[0] = alpha;
  qArm[1] = beta;
  qArm[2] = gamma;
  qArm[3] = delta;
  qArm[4] = epsilon;
  
  return qArm;
}