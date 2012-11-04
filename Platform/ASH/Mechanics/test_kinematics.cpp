#include <stdio.h>
#include "Kinematics.h"

int main()
{
  std::vector<double> r(12); //actuator lengths
  std::vector<double> q(12); // joint positions
  std::vector<double> q0(12);
  std::vector<double> eib(20);
  std::vector<double> eob(20);

  q[0]  = 0.05; //{-0.3-0.1}
  q[1]  = -0.4;//
  q[2]  = .3;
  q[3]  = 0.1;
  q[4]  = -0.20;
  q[5]  = -0.4;
  q[6]  = -0.2;
  q[7]  = 0.1;//-1.5708;
  q[8]  = -.20;
  q[9]  = 0.090;
  q[10] = 1.;
  q[11] = 0.5;
  
  q0=q;
  int error=0;
  Transform t1;
  Transform t2;
  t1=kinematics_forward_l_leg(&q[0]);
  t2=kinematics_forward_r_leg(&q[0]);
  printTransform(t1);
  printTransform(t2);

  std::vector<double> qLeg(6);
  qLeg=kinematics_inverse_r_leg(t2);
  printf("\n");
  printVector(qLeg);
  getchar();
  return 0;
}
