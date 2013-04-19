#ifndef _JOINT_KINEMATICS_H_
#define _JOINT_KINEMATICS_H_

#include <math.h>
#include <vector>
#include "Lut.h"
#include "Transform.h"
#include "fk_tables.h"

const double PI = 2*asin(1);
const double SQRT2 = sqrt(2);

// Define constants for inverse kinematics
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

double distance3d(const double *point1, const double *point2);
std::vector<double> kinematics_forward_joints(const double *r);
std::vector<double> kinematics_inverse_joints(const double *q);

#endif
