#include "Kinematics.h"
#include "Transform.h"
#include "Lut.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

enum {LEG_LEFT = 0, LEG_RIGHT = 1};
enum {ARM_LEFT = 0, ARM_RIGHT = 1};

double distance3d(const double *point1, const double *point2)
{
  double sum = 0;
  for (int i=0; i<3; i++)
    sum = sum + (point1[i] - point2[i])*(point1[i] - point2[i]);
  return sqrt(sum);    
}

std::vector<double>
kinematics_forward_joints(const double *r)  //dereks function to write
{
  /* forward kinematics to convert servo positions to joint angles */
  /* for joints with revolute actuators, joint angle = servo position */
  /* for joints with linear actuators, we use lookup tables  */ 

  std::vector<double> q(12);
  std::vector<double> s(12);

  // translate servo positions to match kinematic coordinates 
  for (int i = 0; i < 12; i++) {
    s[i] = r[i] + servoOffset[i];
  }

  q[0] = s[0]; // l_hip_yaw 
  q[1] = fk_l_hip_roll_lut.interpolate(s[0], s[1], s[2]);
  q[2] = fk_l_hip_pitch_lut.interpolate(s[0], s[1], s[2]);
  q[3] = fk_l_knee_pitch_lut.interpolate(s[3]);
  q[4] = fk_l_ankle_pitch_lut.interpolate(s[4], s[5]);
  q[5] = fk_l_ankle_roll_lut.interpolate(s[4], s[5]);

  q[6] = s[6]; // r_hip_yaw 
  q[7] = -fk_l_hip_roll_lut.interpolate(-s[6], s[7], s[8]);
  q[8] = fk_l_hip_pitch_lut.interpolate(-s[6], s[7], s[8]);
  q[9] = fk_l_knee_pitch_lut.interpolate(s[9]);
  q[10] = fk_l_ankle_pitch_lut.interpolate(s[10], s[11]);
  q[11] = -fk_l_ankle_roll_lut.interpolate(s[10], s[11]);
 
  return q;
}

Transform
kinematics_forward_head(const double *q)
{
  Transform t;
  t = t.translateZ(neckOffsetZ)
    .mDH(0, 0, q[0], 0)
    .mDH(-PI/2, 0, -PI/2+q[1], 0)
    .rotateX(PI/2).rotateY(PI/2);
  return t;
}

Transform
kinematics_forward_l_arm(const double *q)
{
  Transform t;
  t = t.translateY(shoulderOffsetY).translateZ(shoulderOffsetZ)
    .mDH(-PI/2, 0, q[0], 0)
    .mDH(PI/2, 0, PI/2, 0)
    .mDH(PI/2, 0, q[1], upperArmLength)
    .mDH(-PI/2, 0, q[2], 0)
    .mDH(PI/2, 0, 0, lowerArmLength)
    .rotateX(-PI/2).rotateZ(-PI/2)
    .translateX(handOffsetX).translateZ(-handOffsetZ);
  return t;
}

Transform
kinematics_forward_r_arm(const double *q)
{
  Transform t;
  t = t.translateY(-shoulderOffsetY).translateZ(shoulderOffsetZ)
    .mDH(-PI/2, 0, q[0], 0)
    .mDH(PI/2, 0, PI/2, 0)
    .mDH(PI/2, 0, q[1], upperArmLength)
    .mDH(-PI/2, 0, q[2], 0)
    .mDH(PI/2, 0, 0, lowerArmLength)
    .rotateX(-PI/2).rotateZ(-PI/2)
    .translateX(handOffsetX).translateZ(-handOffsetZ);
  return t;
}

Transform
kinematics_forward_l_leg(const double *q) //tested DFL
{
  Transform t;
  t = t.translateY(hipOffsetY)
    .rotateZ(q[0]).rotateX(q[1]).rotateY(q[2])
    .translateZ(-1*thighLength).translateX(kneeOffsetX)
    .rotateY(q[3]).translateZ(-1*tibiaLength)
    .rotateY(q[4]).rotateX(q[5]).translateZ(-1*footHeight);
  return t;
}

Transform
kinematics_forward_r_leg(const double *q) //tested DFL
{
  Transform t;
  t = t.translateY(-1*hipOffsetY)
    .rotateZ(q[6]).rotateX(q[7]).rotateY(q[8])
    .translateZ(-1*thighLength).translateX(kneeOffsetX)
    .rotateY(q[9]).translateZ(-1*tibiaLength)
    .rotateY(q[10]).rotateX(q[11]).translateZ(-1*footHeight);
  return t;
}

std::vector<double>
kinematics_inverse_joints(const double *q)  //dereks code to write
{
  // inverse kinematics to convert joint angles to servo positions
  Transform t;
  std::vector<double> s(12);
  std::vector<double> r(12);
  double actuator_top[12][3];
  double actuator_bot[12][3];

  for (int i = 0; i < 12; i++) {
    s[i] = servoOffset[i];
    memcpy(actuator_top[i], actuatorTop[i], 3*sizeof(double));
    memcpy(actuator_bot[i], actuatorBot[i], 3*sizeof(double));
  }

  // left hip
  t.clear();
  t = t.translateY(links[0][1]).rotateZ(q[0]).rotateX(q[1]).rotateY(q[2]);
  t.apply(actuator_bot[1]);
  t.apply(actuator_bot[2]);
  s[0] = q[0];
  s[1] = distance3d(actuator_top[1], actuator_bot[1]);
  s[2] = distance3d(actuator_top[2], actuator_bot[2]);

  // left knee
  t.clear();
  t = t.translateX(links[1][0]).translateZ(links[1][2]).rotateY(q[3]);
  t.apply(actuator_bot[3]);
  s[3] = distance3d(actuator_top[3], actuator_bot[3]);

  // left ankle
  t.clear();
  t = t.translateX(links[2][0]).translateZ(links[2][2]).rotateY(q[4]).rotateX(q[5]);
  t.apply(actuator_bot[4]);
  t.apply(actuator_bot[5]);
  s[4] = distance3d(actuator_top[4], actuator_bot[4]);
  s[5] = distance3d(actuator_top[5], actuator_bot[5]);

  // right hip
  t.clear();
  t = t.translateY(links[3][1]).rotateZ(q[6]).rotateX(q[7]).rotateY(q[8]);
  t.apply(actuator_bot[7]);
  t.apply(actuator_bot[8]);
  s[6] = q[6];
  s[7] = distance3d(actuator_top[7], actuator_bot[7]);
  s[8] = distance3d(actuator_top[8], actuator_bot[8]);

  // right knee
  t.clear();
  t = t.translateX(links[4][0]).translateZ(links[4][2]).rotateY(q[9]);
  t.apply(actuator_bot[9]);
  s[9] = distance3d(actuator_top[9], actuator_bot[9]);

  // right ankle
  t.clear();
  t = t.translateX(links[5][0]).translateZ(links[5][2]).rotateY(q[10]).rotateX(q[11]);
  t.apply(actuator_bot[10]);
  t.apply(actuator_bot[11]);
  s[10] = distance3d(actuator_top[10], actuator_bot[10]);
  s[11] = distance3d(actuator_top[11], actuator_bot[11]);
  
  // translate servo positions to match zero pose coordinates
  for (int i = 0; i < 12; i++) {
    r[i] = s[i] - servoOffset[i];
  }
  return r;
}

std::vector<double>
kinematics_inverse_arm(Transform trArm, int arm)
{
  std::vector<double> qArm(6);
  return qArm;
}

std::vector<double>
kinematics_inverse_l_arm(Transform trArm)
{
  return kinematics_inverse_arm(trArm, ARM_LEFT);
}

std::vector<double>
kinematics_inverse_r_arm(Transform trArm)
{
  return kinematics_inverse_arm(trArm, ARM_RIGHT);
}

std::vector<double>
kinematics_inverse_leg(Transform trLeg, int leg)
{
  std::vector<double> qLeg(6);
  Transform trInvLeg = inv(trLeg);

  // Hip Offset vector in Torso frame
  double xHipOffset[3];
  if (leg == LEG_LEFT) {
    xHipOffset[0] = 0;
    xHipOffset[1] = hipOffsetY;
    xHipOffset[2] = -hipOffsetZ;
  }
  else {
    xHipOffset[0] = 0;
    xHipOffset[1] = -hipOffsetY;
    xHipOffset[2] = -hipOffsetZ;
  }

  // Hip Offset in Leg frame
  double xLeg[3];
  for (int i = 0; i < 3; i++)
    xLeg[i] = xHipOffset[i];
  trInvLeg.apply(xLeg); //gets coordinates of body in the leg frame
  xLeg[2] -= footHeight; //moves reference frame to ankle

  // Knee pitch
  double dLeg = xLeg[0]*xLeg[0] + xLeg[1]*xLeg[1] + xLeg[2]*xLeg[2]; //leg length from ankle to hip

  double cKnee = .5*(dLeg-dTibia*dTibia-dThigh*dThigh)/(dTibia*dThigh);
  if (cKnee > 1) cKnee = 1;
  if (cKnee < -1) cKnee = -1;
  double kneePitch = acos(cKnee);

  // Angle pitch and roll
  double ankleRoll = atan2(xLeg[1], xLeg[2]);
  double lLeg = sqrt(dLeg);
  if (lLeg < 1e-16) lLeg = 1e-16;
  double pitch0 = asin(dThigh*sin(kneePitch)/lLeg);
  double anklePitch = asin(-xLeg[0]/lLeg) - pitch0;

  Transform rHipT = trLeg;
  rHipT = rHipT.rotateX(-ankleRoll).rotateY(-anklePitch-kneePitch);

  double hipYaw = atan2(-rHipT(0,1), rHipT(1,1));
  double hipRoll = asin(rHipT(2,1));
  double hipPitch = atan2(-rHipT(2,0), rHipT(2,2));

  // Need to compensate for KneeOffsetX:
  qLeg[0] = hipYaw;
  qLeg[1] = hipRoll;
  qLeg[2] = hipPitch+aThigh; //changed sign on aThigh
  qLeg[3] = kneePitch-aThigh; //changed sign on aThigh
  qLeg[4] = anklePitch;
  qLeg[5] = ankleRoll;
  return qLeg;
}

std::vector<double>
kinematics_inverse_l_leg(Transform trLeg)
{
  return kinematics_inverse_leg(trLeg, LEG_LEFT);
}

std::vector<double>
kinematics_inverse_r_leg(Transform trLeg)
{
  return kinematics_inverse_leg(trLeg, LEG_RIGHT);
}
