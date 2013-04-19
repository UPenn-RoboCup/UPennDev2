#include "joint_kinematics.h"
#include "Transform.h"
#include "Lut.h"
#include <math.h>
#include <string.h>

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
