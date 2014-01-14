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

// DH transform params: (alpha, a, theta, d)
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

std::vector<double> YouBot_kinematics_inverse_arm(const double *tr) {
  // Grab the position
  double x = tr[0];
  double y = tr[1];
  double z = tr[2];
  // Grab the pitch desired
  double p = tr[4];
  
  // Remove the height of the body
  z -= baseLength;
  
  // Remove rotation of the shoulder yaw
  double dist2 = x*x + y*y;
  double dist = sqrt(dist2);
  double yaw = atan2(y,x);
  // x is the distance now
  x = dist;
  
  // Given the pitch, find the effective position
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
  qArm[1] = -1*shoulderPitch;
  qArm[2] = PI - elbow;
  qArm[3] = (shoulderPitch + qArm[2]) - p;
  qArm[4] = 0;
  
  return qArm;
  
}