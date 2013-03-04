#include "Kinematics.h"

enum {LEG_LEFT = 0, LEG_RIGHT = 1};
enum {ARM_LEFT = 0, ARM_RIGHT = 1};

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
    .rotateZ(q[0]).rotateX(q[1]).rotateY(q[2])
    .translateZ(-1*thighLength).translateX(kneeOffsetX)
    .rotateY(q[3]).translateZ(-1*tibiaLength)
    .rotateY(q[4]).rotateX(q[5]).translateZ(-1*footHeight);
  return t;
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
