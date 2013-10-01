/*
 * MotionStatus.cpp
 *
 *  Created on: 2013. 1. 17.
 *      Author: hjsong
 */

#include "framework/motion/motionstatus.h"

using namespace Thor;
//
std::vector<JointData> MotionStatus::m_CurrentJoints;

EnableList MotionStatus::m_EnableList[MotionStatus::MAXIMUM_NUMBER_OF_JOINTS];

double MotionStatus::FB_GYRO(0);
double MotionStatus::RL_GYRO(0);
double MotionStatus::FB_ACCEL(0);
double MotionStatus::RL_ACCEL(0);

int MotionStatus::FALLEN(0);

double MotionStatus::EulerAngleX(0);
double MotionStatus::EulerAngleY(0);
double MotionStatus::EulerAngleZ(0);

double MotionStatus::R_LEG_FSR1(0);
double MotionStatus::R_LEG_FSR2(0);
double MotionStatus::R_LEG_FSR3(0);
double MotionStatus::R_LEG_FSR4(0);

double MotionStatus::L_LEG_FSR1(0);
double MotionStatus::L_LEG_FSR2(0);
double MotionStatus::L_LEG_FSR3(0);
double MotionStatus::L_LEG_FSR4(0);

double MotionStatus::R_LEG_FX(0);
double MotionStatus::R_LEG_FY(0);
double MotionStatus::R_LEG_FZ(0);
double MotionStatus::R_LEG_TX(0);
double MotionStatus::R_LEG_TY(0);
double MotionStatus::R_LEG_TZ(0);

double MotionStatus::L_LEG_FX(0);
double MotionStatus::L_LEG_FY(0);
double MotionStatus::L_LEG_FZ(0);
double MotionStatus::L_LEG_TX(0);
double MotionStatus::L_LEG_TY(0);
double MotionStatus::L_LEG_TZ(0);


double MotionStatus::R_ARM_FSR1(0);
double MotionStatus::R_ARM_FSR2(0);
double MotionStatus::R_ARM_FSR3(0);
double MotionStatus::R_ARM_FSR4(0);

double MotionStatus::L_ARM_FSR1(0);
double MotionStatus::L_ARM_FSR2(0);
double MotionStatus::L_ARM_FSR3(0);
double MotionStatus::L_ARM_FSR4(0);

double MotionStatus::R_ARM_FX(0);
double MotionStatus::R_ARM_FY(0);
double MotionStatus::R_ARM_FZ(0);
double MotionStatus::R_ARM_TX(0);
double MotionStatus::R_ARM_TY(0);
double MotionStatus::R_ARM_TZ(0);

double MotionStatus::L_ARM_FX(0);
double MotionStatus::L_ARM_FY(0);
double MotionStatus::L_ARM_FZ(0);
double MotionStatus::L_ARM_TX(0);
double MotionStatus::L_ARM_TY(0);
double MotionStatus::L_ARM_TZ(0);


double MotionStatus::lidar_data[2][1080];
