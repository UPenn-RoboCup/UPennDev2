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

/*
std::vector<double> YouBot_kinematics_inverse_arm(Transform trArm) {
  double alpha, beta, gamma, delta, epsilon;
  
  //printTransform(trArm);
  
  double z = trArm(0,3);
  double x = trArm(1,3);
  double y = trArm(2,3);
  
  printf("\n\tx: %lf, y: %lf, z: %lf\n",x,y,z);
  
  double x1 = sqrt(x*x + z*z);
  double y1 = y + wristLength + handLength - baseLength;
  
  printf("\n\tx1: %lf, y1: %lf\n",x1,y1);
  
  double a = lowerArmLength;
  double b = upperArmLength;
  double c = sqrt(x1*x1 + y1*y1);
  
  printf("\n\ta: %lf, b: %lf, c: %lf\n",a,b,c);
  
  // Singularity treatment
  if(x1>1e-6||x1<-1e-6){
    alpha = - asin( z / x1 );
  } else {
    alpha = 0;
  }
  
  printf("\n\tbeta ratio %lf\n",(a*a + c*c - b*b) / (2.0*a*c));
  
  beta = -(M_PI_2 - acos( (a*a + c*c - b*b) / (2.0*a*c) ) - atan( y1/x1 ));
  gamma = -(M_PI - acos( (a*a + b*b - c*c) / (2.0*a*b) ));
  delta = -(M_PI + (beta + gamma));
  epsilon = M_PI_2 + alpha;
  
  std::vector<double> qArm(7);
  qArm[0] = alpha;
  qArm[1] = beta;
  qArm[2] = gamma;
  qArm[3] = delta;
  qArm[4] = epsilon;
  
  return qArm;
}
*/