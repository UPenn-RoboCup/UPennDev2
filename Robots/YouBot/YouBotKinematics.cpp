/*
(c) 2014 Stephen McGill
*/
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

Transform YouBot_kinematics_forward_arm(const double *q) {
  Transform t;
  t = t.translateZ(baseLength)
    .rotateZ(q[0])
    .rotateY(q[1])
    .translateZ(lowerArmLength)
    .rotateY(q[2])
    .translateZ(upperArmLength)
    .rotateY(q[3])
    .translateZ(wristLength+handLength)
    .rotateZ(q[4]);
  return t;
}

#ifdef TORCH
std::vector<double> YouBot_kinematics_inverse_arm(const THDoubleTensor * tr) {
#else
std::vector<double> YouBot_kinematics_inverse_arm(const double *tr) {
#endif
  double x, y, z, yaw, p, hand_yaw;
  #ifdef TORCH
  x = THTensor_fastGet2d( tr, 0, 3 );
  y = THTensor_fastGet2d( tr, 1, 3 );
  z = THTensor_fastGet2d( tr, 2, 3 );
  // Grab the "pitch" desired (ZYZ where Y is "pitch")
  double tmp1 = THTensor_fastGet2d( tr, 0, 2 );
  double tmp2 = THTensor_fastGet2d( tr, 1, 2 );
  p = atan2(
    sqrt(tmp1*tmp1 + tmp2*tmp2),
    THTensor_fastGet2d( tr, 2, 2 )
  );

  // Grab also the "yaw" of the gripper (ZYZ, where 2nd Z is yaw)
  hand_yaw = atan2(
    THTensor_fastGet2d( tr, 2, 1 ),
    -THTensor_fastGet2d( tr, 2, 0 )
  );


  //printf("xyz: %lf %lf %lf\n",x,y,z);
  //printf("zyz: 0 %lf %lf\n",p,hand_yaw);
  #else
  // Grab the position
  x = tr[0];
  y = tr[1];
  z = tr[2];
  // This is RPY pitch... hacked in lua to be ZYZ Y as "pitch"
  p = tr[4];
  // yaw is second Z
  hand_yaw = tr[5];
  #endif
  
  // Remove the height of the body
  z -= baseLength;
  
  // Remove rotation of the shoulder yaw
  double dist2 = x*x + y*y;
  double dist = sqrt(dist2);
  yaw = atan2(y,x);
  // x is the distance now
  x = dist;
  
  // Given the "pitch", find the effective position
  double dx = x - gripperLength * sin(p);
  double dz = z - gripperLength * cos(p);
  
  //printf("\n\tdx: %lf, dz: %lf\n",dx,dz);
  
  double dr2 = dx*dx+dz*dz;
  double dr  = sqrt(dr2);
  double elevation = PI/2 - atan2(dz,dx);
  
  //printf("\n\televation: %lf\n",elevation);
  
  // Find the elbow, given the effective position
  // Law of cosines
  double elbow_help = upperArmLength*upperArmLength + lowerArmLength*lowerArmLength - dr2;
  elbow_help /= 2*upperArmLength*lowerArmLength;
  //printf("\n\telbow_help: %lf\n",elbow_help);
  if(elbow_help<-1)
    elbow_help = -1;
  else if(elbow_help>1)
    elbow_help = 1;
  
  double elbow = acos(elbow_help);
  //printf("\n\telbow: %lf\n",elbow);
  
  // Find how much to rotate the shoulder pitch
  // Law of sines
  double ratio = upperArmLength * sin(elbow) / dr;
  
  //printf("\n\tRatio: %lf\n",ratio);
  
  double effective_elevation = asin(ratio);
  double shoulderPitch = elevation - effective_elevation;
  
  //printf("\n\teffective_elevation: %lf\n",effective_elevation);
  
  // Output to joint angles
  std::vector<double> qArm(5);
  qArm[0] = yaw;
  qArm[1] = shoulderPitch;
  qArm[2] = PI - elbow;
  qArm[3] = p - (shoulderPitch + qArm[2]);
  //qArm[4] = hand_yaw;
  qArm[4] = 0;
  
  return qArm;
  
}
