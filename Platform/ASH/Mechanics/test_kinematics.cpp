#include <stdio.h>
#include "Kinematics.h"
//#include "fk_tables.h"

int main()
{
/*
  r[0]  = -0.05; 
  r[1]  = 0.3; 
  r[2]  = 0.28; 
  r[3]  = 0.27; 
  r[4]  = 0.30; 
  r[5]  = 0.29; 
  
  r[6]  = 0.2; 
  r[7]  = 0.27; 
  r[8]  = 0.28; 
  r[9]  = 0.3; 
  r[10] = 0.29;  
  r[11] = 0.3; 
*/
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
  
  /* q to r */
  //r = kinematics_inverse_joints(&q[0]);  //joints to actuators
  //printVector(r);
  //printf("\n");
  
    /* r to q */
  //q = kinematics_forward_joints(&r[0],&error); //actuators to joints
  /*printVector(q);
  printf("\n");
  printf(" error= %d", error);*/

  //double ca = sin(0);
  //printf("\n %.5g ", ca);
  /*for(int i=0;i<10;i++){
      q0[0]=-0.3+0.04*i;    
      //printVector(q0);
      r = kinematics_inverse_joints(&q0[0]);  //joints to actuators
      q = kinematics_forward_joints(&r[0]); //actuators to joints
      //printVector(q);
      eib[i]=q[1]-q0[1];
      eob[i]=q[2]-q0[2];
      printf("\n q[0]= %.4g", q0[0]); printf( " roll= %.4g ", eib[i]);  printf(" pitch %.4g", eob[i]);   
  }*/
  getchar();
  return 0;
}
