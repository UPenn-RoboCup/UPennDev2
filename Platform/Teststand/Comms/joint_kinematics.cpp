#include "joint_kinematics.h"
#include "Transform.h"
#include "Lut.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

// joint_kinematics.h : kinematics interface for actuator teststand 
///////////////////////////////////////////////////////////////////////////

double distance3d(const double *point1, const double *point2)
{
  double sum = 0;
  for (int i=0; i<3; i++)
    sum = sum + (point1[i] - point2[i])*(point1[i] - point2[i]);
  return sqrt(sum);    
}

std::vector<double>
kinematics_forward_joints(const double *r)
{
  /* forward kinematics to convert servo positions to joint angles */
  /* for joints with revolute actuators, joint angle = servo position */
  /* for joints with linear actuators, we use lookup tables  */ 

  // TODO: make these calculations accurate to the teststand

  std::vector<double> q(1);
  std::vector<double> s(1);

  s[0] = r[0] + home_offset[0];
  q[0] = fk_l_knee_pitch_lut.interpolate(s[0]);
  return q;
}

std::vector<double>
kinematics_inverse_joints(const double *q)
{
  /* inverse kinematics to convert joint angles to servo positions */

  // TODO: make these calculations accurate to the teststand

  Transform t;
  std::vector<double> s(1);
  std::vector<double> r(1);
  double ujoint_top[3];
  double ujoint_bot[3];
  
  memcpy(ujoint_top, actuator_top[0], 3*sizeof(double));
  memcpy(ujoint_bot, actuator_bot[0], 3*sizeof(double));
 
  // left knee
  t.clear();
  t = t.translateX(links[0][0]).rotateY(q[0]);
  t.apply(ujoint_bot);
  s[0] = distance3d(ujoint_top, ujoint_bot);
  r[0] = s[0] - home_offset[0];
  return r;
}
