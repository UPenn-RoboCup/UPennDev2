#include <stdio.h>
#include "Kinematics.h"
#include "Statics.h"

int main()
{
  std::vector<double> r(12); //actuator lengths
  std::vector<double> q(12); // joint positions
  std::vector<double> v(12); // actuator forces
  std::vector<double> t(12);
  std::vector<double> t2(12);
  std::vector<double> act_len(12);
  Transform tjoint;
  
  q[0]  = 0; //{-0.3-0.1}
  q[1]  = 0;//
  q[2]  = 0;
  q[3]  = 0;
  q[4]  = 0;
  q[5]  = 0;
  q[6]  = 0;
  q[7]  = 0;//-1.5708;
  q[8]  = 0;
  q[9]  = 0;
  q[10] = 0;
  q[11] = 0.2;
  
  v[0]  = 1;
  v[1]  = 1;
  v[2]  = 1;
  v[3]  = 1;
  v[4]  = 1;
  v[5]  = 1;
  v[6]  = 1;
  v[7]  = 1;
  v[8]  = 1;
  v[9]  = 1;
  v[10] = 1;
  v[11] = 1;
  
  t[0]  = 0;
  t[1]  = 1;
  t[2]  = 1;
  t[3]  = 1;
  t[4]  = 1;
  t[5]  = 1;
  t[6]  = 0;
  t[7]  = 1;
  t[8]  = 1;
  t[9]  = 1;
  t[10] = 1;
  t[11] = 1;
  
  r=kinematics_inverse_joints(&q[0]);
  for (int i = 0; i < 12; i++) {
    act_len[i] = r[i] + servoOffset[i];
  }
  printf("actuator lengths\n");
  printVector(act_len);
  printf("\n\n");
  
  t2=statics_forward_joints(&v[0], &q[0], &r[0]);
  printf("t2\n");
  printVector(t2);
  printf("\n\n");
  v=statics_inverse_joints(&t2[0], &q[0], &r[0]);
  printf("v\n");
  printVector(v);
      
  /*tjoint.translateY(links[0][1]).rotateZ(q[0]).rotateX(q[1]).rotateY(q[2]);  
  printTransform(tjoint);
  printf("\n");
  t=find_unit_torque (tjoint, 1, 0, 0.2954, 1);
  printf("\n");
  printVector(t);
  printf("\n");*/

  /*std::vector<double> qLeg(6);
  qLeg=kinematics_inverse_r_leg(t2);
  printf("\n");
  printVector(qLeg);*/

  getchar();
  return 0;
}
