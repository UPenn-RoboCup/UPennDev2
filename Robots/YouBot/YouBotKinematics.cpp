/*
(c) 2014 Stephen G. McGill
Kinematics for KUKA YouBot's 5 DOF arm
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
  t = t
    .rotateZ(q[0])
    .translateX(baseLength) // do not use for now
    .rotateY(q[1])
    .translateZ(lowerArmLength)
    .rotateY(q[2])
    .translateZ(upperArmLength)
    .rotateY(q[3])
    .translateZ(wristLength+handLength)
    .rotateZ(q[4]);
  return t;
}

// Given the appropriate yaw, just get the angles in the xz plane
// Input: x and z position, pitch angle
// Output: Vector of 3 pitch angles
std::vector<double> get_xz_angles(double x, double z, double p){
  // Given the "pitch", find the effective position
  double dx = x - gripperLength * sin(p);
  double dz = z - gripperLength * cos(p);
  
  //printf("\n\tdx: %lf, dz: %lf\n",dx,dz);
  
  double dr2 = dx*dx+dz*dz;
  double dr  = sqrt(dr2);
  
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
  
  //printf("\n\teffective_elevation: %lf\n",effective_elevation);
  double elevation = PI/2 - atan2(dz,dx);
  double shoulderPitch = elevation - effective_elevation;

  // Form the vector
  std::vector<double> xz(3);
  xz[0] = shoulderPitch;
  xz[1] = PI - elbow;
  xz[2] = p - (shoulderPitch + xz[1]);

  return xz;
}

// Inverse given a transform
std::vector<double> YouBot_kinematics_inverse_arm(Transform tr) {
  double x, y, z, yaw, p, hand_yaw;

  // Grab the position
  x = tr(0,3);
  y = tr(1,3);
  z = tr(2,3);
  // Grab the "pitch" desired (ZYZ where Y is "pitch")
  double tmp1 = tr( 0, 2 );
  double tmp2 = tr( 1, 2 );
  p = atan2(
    sqrt(tmp1*tmp1 + tmp2*tmp2),
    tr( 2, 2 )
  );
  printf("p: %lf\n",p);
  // Grab also the "yaw" of the gripper (ZYZ, where 2nd Z is yaw)
  hand_yaw = atan2(
    tr( 2, 1 ),
    -1*tr( 2, 0 )
  );

  //printf("xyz: %lf %lf %lf\n",x,y,z);
  //printf("zyz: %lf %lf %lf\n",0.0,p,hand_yaw);

  // Remove rotation of the shoulder yaw
  double dist2 = x*x + y*y;
  double dist = sqrt(dist2);
  yaw = atan2(y,x);
  // x is the distance now, less the offset
  x = dist - baseLength;

  // Grab the XZ plane angles
  std::vector<double> xz = get_xz_angles( x, z, p );

  // Output to joint angles
  std::vector<double> qArm(5);
  qArm[0] = yaw;
  qArm[1] = xz[0];
  qArm[2] = xz[1];
  qArm[3] = xz[2];
  qArm[4] = hand_yaw;// + yaw * cos(p);
	//printf("hand_yaw: %lf, yaw: %lf, p: %lf, offset: %lf\n",hand_yaw,yaw,p,yaw * cos(p));
  
  return qArm;
  
}

// Inverse given only a position
std::vector<double> YouBot_kinematics_inverse_arm_position(double x, double y, double z) {

  // Find the perfect pitch

  // Remove rotation of the shoulder yaw
  double dist2 = x*x + y*y;
  double dist = sqrt(dist2);
  double yaw = atan2(y,x);
  // x is the distance now, less the offset
  x = dist - baseLength;

  // TODO: Make a simple optimization routine
  // How much the arm must reach
  double dr2 = x*x+z*z;
  double dr  = sqrt(dr2);
  double p;
  if(dr<.35) {
    // Optimization routine in tight spaces
    // Basically loop between 90 and 180 degrees
    // TODO: Could in clude the *current* pitch angle
    double ratio = (dr-.1)/.3;
    if(ratio<0) ratio=0;
    p = ratio * PI/2 + (1-ratio)*PI*125/180;
    printf("pp: %lf, ratio: %lf\n",p,ratio);
  } else {
    // The angle of attack is our perfect pitch
    p = atan2(x,z);
  }
  // End perfect pitch calculation

  // Grab the XZ plane angles
  std::vector<double> xz = get_xz_angles( x, z, p );

  // Output to joint angles
  std::vector<double> qArm(5);
  qArm[0] = yaw;
  qArm[1] = xz[0];
  qArm[2] = xz[1];
  qArm[3] = xz[2];
  qArm[4] = 0; // Go to zero here, but this is arbitrary

  return qArm;
}
