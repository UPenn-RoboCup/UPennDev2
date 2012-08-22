#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <math.h>
#include <vector>
#include "Lut.h"
#include "Transform.h"
#include "fk_tables.h"

const double PI = 2*asin(1);
const double SQRT2 = sqrt(2);

// Define constants for inverse kinematics
const double neckOffsetZ = 0;
const double neckOffsetX = 0;
const double shoulderOffsetX = 0;
const double shoulderOffsetY = 0;
const double shoulderOffsetZ = 0;
const double handOffsetX = 0;
const double handOffsetZ = 0;
const double upperArmLength = 0;
const double lowerArmLength = 0;
const double hipOffsetY = .097; //updated to SAFFiR  DFL  120218
const double hipOffsetZ = .00;
const double hipOffsetX = .000;
const double thighLength = .379025;
const double tibiaLength = .3800;
const double footHeight = .0487;
const double kneeOffsetX = .00255;
const double dThigh = sqrt(thighLength*thighLength+kneeOffsetX*kneeOffsetX);
const double aThigh = atan(kneeOffsetX/thighLength);
const double dTibia = tibiaLength; //changed to eliminate kneeOffsetX for SAFFiR
const double aTibia = 0; //changed to eliminate kneeOffsetX for SAFFiR
const double servoOffset[] = {
  0.0, 0.2954, 0.3025, 0.2566, 0.3167, 0.3166, // l_leg
  0.0, 0.2954, 0.3025, 0.2566, 0.3167, 0.3166  // r_leg
};
const double links [6][3] = {
  {0, 0.097, 0}, // l_leg
  {-0.00255, 0, -0.379},
  {-0.005, 0, -0.380},
  {0, -0.097, 0}, // r_leg
  {-0.00255, 0, -0.379},
  {-0.005, 0, -0.380}
};
const double actuatorTop [12][3] = {
  {0, 0, 0}, // l_leg
  {-0.06300, 0.03225, 0.29930},
  {-0.06300, 0.11363, 0.30237},
  {0.07100, -0.00025, -0.07470},
  {-0.06105, -0.03875, -0.05470},
  {-0.06105, 0.03875, -0.05480},
  {0, 0, 0}, // r_leg
  {-0.06300, -0.03225, 0.29930},
  {-0.06300, -0.11363, 0.30237},
  {0.07100, -0.00025, -0.07470},
  {-0.06105, 0.03875, -0.05470},
  {-0.06105, -0.03875, -0.05480}
};
const double actuatorBot [12][3] = {
  {0, 0, 0}, // l_leg
  {-0.071, -0.068, 0.004},
  {-0.071, 0.066, 0.004},
  {0.05745, 0.00, 0.04821},
  {-0.07250, -0.03755, 0.00900},
  {-0.07250, 0.03744, 0.00900},
  {0, 0, 0}, // r_leg
  {-0.071, 0.068, 0.004},
  {-0.071, -0.066, 0.004},
  {0.05745, 0.00, 0.04821},
  {-0.07250, 0.03755, 0.00900},
  {-0.07250, -0.03744, 0.00900}
};

// Define lookup tables for forward kinematics 
const Lut3d fk_l_hip_roll_lut(fk_l_hip_roll_min, fk_l_hip_roll_max, 
  fk_l_hip_roll_dim, &fk_l_hip_roll_data[0][0][0]);
const Lut3d fk_l_hip_pitch_lut(fk_l_hip_pitch_min, fk_l_hip_pitch_max, 
  fk_l_hip_pitch_dim, &fk_l_hip_pitch_data[0][0][0]);
const Lut1d fk_l_knee_pitch_lut(fk_l_knee_pitch_min, fk_l_knee_pitch_max, 
  fk_l_knee_pitch_dim, &fk_l_knee_pitch_data[0]);
const Lut2d fk_l_ankle_roll_lut(fk_l_ankle_roll_min, fk_l_ankle_roll_max, 
  fk_l_ankle_roll_dim, &fk_l_ankle_roll_data[0][0]);
const Lut2d fk_l_ankle_pitch_lut(fk_l_ankle_pitch_min, fk_l_ankle_pitch_max, 
  fk_l_ankle_pitch_dim, &fk_l_ankle_pitch_data[0][0]);

void printTransform(Transform tr);
void printVector(std::vector<double> v);
double distance3d(const double *point1, const double *point2);

Transform kinematics_forward_head(const double *q);
Transform kinematics_forward_l_arm(const double *q);
Transform kinematics_forward_r_arm(const double *q);
Transform kinematics_forward_l_leg(const double *q);
Transform kinematics_forward_r_leg(const double *q);
std::vector<double> kinematics_forward_joints(const double *r);

std::vector<double> kinematics_inverse_r_arm(const Transform trArm);
std::vector<double> kinematics_inverse_l_arm(const Transform trArm);
std::vector<double> kinematics_inverse_r_leg(const Transform trLeg);
std::vector<double> kinematics_inverse_l_leg(const Transform trLeg);
std::vector<double> kinematics_inverse_joints(const double *q);

#endif
