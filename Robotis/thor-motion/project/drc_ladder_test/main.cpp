/*
 * main.cpp
 *
 *  Created on: 2013. 1. 3.
 *      Author: zerom
 */

#include <stdio.h>
#include <stdio_ext.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <getopt.h>
#include <string.h>
#include <termios.h>
#include <term.h>
#include <ncurses.h>
#include <libgen.h>
//#include "streamer/mjpg_streamer.h"
//#include "framework/motion/modules/testmodule.h"
#include "framework/Thor.h"

#define INI_FILE_PATH "config.ini"

#define PI (3.14159265)
#define ANG2RAD PI/180.0

using namespace Thor;


StepData g_stp0, g_stp1, g_stp2, g_stp3, g_stpe;

double g_stride_length = 1.0;
double g_side_stride_length = 1.0;
double g_stride_degree = 1.0;
double g_stride_radian = g_stride_degree*ANG2RAD;

double g_dsp = 0.2;
double g_period = 850.0;

double g_se_time = 2000.0;
double g_foot_height = 40.0;
double g_foot_roll = 0.0, g_foot_pitch = 0.0, g_foot_yaw = 0.0;
//int g_mf_onestep[2];

double g_body_height = 650.0;
double g_y_zmp_convergence = 0.0;

int g_first_foot_move = LFootMove;

int g_phase = 0;

void UpdateTranslateStepData(int first_moving_foot, double fb_stride_length, double rl_stride_length)
{
	double y_pos = 105.0;
	double l_fb_stride_length = fb_stride_length;
	double l_rl_stride_length = rl_stride_length;
	double l_stride_radian = 0.0;
	double l_dsp = g_dsp;
	double l_period = g_period;
	double l_se_time = g_se_time;
	double l_foot_height = g_foot_height;
	double l_foot_roll  = g_foot_roll;
	double l_foot_pitch = g_foot_pitch;
	double l_foot_yaw   = g_foot_yaw;
	double l_body_height = g_body_height;

	//printf("l_foot_pitch %f\n", l_foot_pitch);
	//printf("g_body : %f   l_body : %f    ref : %f", g_body_height, l_body_height, RecursiveWalking::GetInstance()->m_ReferenceGtoBodyPosition.z);

	g_stp0.PositionData.bMovingFoot = NFootMove;                g_stp0.PositionData.dFootHeight = 0.0;               g_stp0.PositionData.dZ_Swap_Amplitude = 0.0;
	g_stp0.PositionData.dShoulderSwingGain = 0.05;              g_stp0.PositionData.dElbowSwingGain = 0.1;
	g_stp0.PositionData.stRightFootPosition.x = 0.0;            g_stp0.PositionData.stRightFootPosition.y = -y_pos;           g_stp0.PositionData.stRightFootPosition.z = 0.0;
	g_stp0.PositionData.stRightFootPosition.roll = l_foot_roll; g_stp0.PositionData.stRightFootPosition.pitch = l_foot_pitch; g_stp0.PositionData.stRightFootPosition.yaw = l_foot_yaw;
	g_stp0.PositionData.stBodyPosition.z = l_body_height;
	g_stp0.PositionData.stBodyPosition.roll = 0.0;             g_stp0.PositionData.stBodyPosition.pitch = 0.0;               g_stp0.PositionData.stBodyPosition.yaw = 0.0;
	g_stp0.PositionData.stLeftFootPosition.x = 0.0;            g_stp0.PositionData.stLeftFootPosition.y = y_pos;             g_stp0.PositionData.stLeftFootPosition.z = 0.0;
	g_stp0.PositionData.stLeftFootPosition.roll = l_foot_roll; g_stp0.PositionData.stLeftFootPosition.pitch = l_foot_pitch;  g_stp0.PositionData.stLeftFootPosition.yaw = l_foot_yaw;

	g_stp0.TimeData.bWalkingState = InWalking;
	g_stp0.TimeData.dAbsStepTime = l_se_time;      g_stp0.TimeData.dDSPratio = l_dsp;
	g_stp0.TimeData.sigmoid_ratio_x = 1.0;         g_stp0.TimeData.sigmoid_ratio_y = 1.0;          g_stp0.TimeData.sigmoid_ratio_z = 1.0;
	g_stp0.TimeData.sigmoid_ratio_roll = 1.0;      g_stp0.TimeData.sigmoid_ratio_pitch = 1.0;      g_stp0.TimeData.sigmoid_ratio_yaw = 1.0;
	g_stp0.TimeData.sigmoid_distortion_x = 1.0;    g_stp0.TimeData.sigmoid_distortion_y = 1.0;     g_stp0.TimeData.sigmoid_distortion_z = 1.0;
	g_stp0.TimeData.sigmoid_distortion_roll = 1.0; g_stp0.TimeData.sigmoid_distortion_pitch = 1.0; g_stp0.TimeData.sigmoid_distortion_yaw = 1.0;


	if(first_moving_foot == RFootMove) {
		g_stp1.PositionData.bMovingFoot = RFootMove;        g_stp1.PositionData.dFootHeight = l_foot_height;              g_stp1.PositionData.dZ_Swap_Amplitude = 10.0;
		g_stp1.PositionData.dShoulderSwingGain = 0.05;      g_stp1.PositionData.dElbowSwingGain = 0.1;

		g_stp1.PositionData.stRightFootPosition.x = l_fb_stride_length; g_stp1.PositionData.stRightFootPosition.y = -y_pos + l_rl_stride_length; g_stp1.PositionData.stRightFootPosition.z = -l_fb_stride_length*tan(l_foot_pitch);
		g_stp1.PositionData.stRightFootPosition.roll = l_foot_roll;     g_stp1.PositionData.stRightFootPosition.pitch = l_foot_pitch;            g_stp1.PositionData.stRightFootPosition.yaw = l_foot_yaw;

		g_stp1.PositionData.stBodyPosition.z = l_body_height - 0.5*l_fb_stride_length*tan(l_foot_pitch);
		g_stp1.PositionData.stBodyPosition.roll = 0.0;      g_stp1.PositionData.stBodyPosition.pitch = 0.0;      g_stp1.PositionData.stBodyPosition.yaw = 0.0;

		g_stp1.PositionData.stLeftFootPosition.x = 0.0;             g_stp1.PositionData.stLeftFootPosition.y = y_pos;            g_stp1.PositionData.stLeftFootPosition.z = 0.0;
		g_stp1.PositionData.stLeftFootPosition.roll = l_foot_roll;  g_stp1.PositionData.stLeftFootPosition.pitch = l_foot_pitch; g_stp1.PositionData.stLeftFootPosition.yaw = l_foot_yaw;

		g_stp1.TimeData.bWalkingState = InWalking;
		g_stp1.TimeData.dAbsStepTime = l_se_time + l_period*1.0;         g_stp1.TimeData.dDSPratio = l_dsp;
		g_stp1.TimeData.sigmoid_ratio_x = 1.0;         g_stp1.TimeData.sigmoid_ratio_y = 1.0;    	   g_stp1.TimeData.sigmoid_ratio_z = 1.0;
		g_stp1.TimeData.sigmoid_ratio_roll = 1.0;      g_stp1.TimeData.sigmoid_ratio_pitch = 1.0; 	   g_stp1.TimeData.sigmoid_ratio_yaw = 1.0;
		g_stp1.TimeData.sigmoid_distortion_x = 1.0;    g_stp1.TimeData.sigmoid_distortion_y = 1.0;     g_stp1.TimeData.sigmoid_distortion_z = 1.0;
		g_stp1.TimeData.sigmoid_distortion_roll = 1.0; g_stp1.TimeData.sigmoid_distortion_pitch = 1.0; g_stp1.TimeData.sigmoid_distortion_yaw = 1.0;



		g_stp2.PositionData.bMovingFoot = LFootMove;        g_stp2.PositionData.dFootHeight = l_foot_height;              g_stp2.PositionData.dZ_Swap_Amplitude = 10.0;
		g_stp2.PositionData.dShoulderSwingGain = 0.05;      g_stp2.PositionData.dElbowSwingGain = 0.1;

		g_stp2.PositionData.stRightFootPosition.x = l_fb_stride_length;  g_stp2.PositionData.stRightFootPosition.y = -y_pos + l_rl_stride_length;  g_stp2.PositionData.stRightFootPosition.z = -l_fb_stride_length*tan(l_foot_pitch);
		g_stp2.PositionData.stRightFootPosition.roll = l_foot_roll;      g_stp2.PositionData.stRightFootPosition.pitch = l_foot_pitch;             g_stp2.PositionData.stRightFootPosition.yaw = l_foot_yaw;

		g_stp2.PositionData.stBodyPosition.z = l_body_height - l_fb_stride_length*tan(l_foot_pitch);
		g_stp2.PositionData.stBodyPosition.roll = 0.0;    g_stp2.PositionData.stBodyPosition.pitch = 0.0;      g_stp2.PositionData.stBodyPosition.yaw = 0.0;

		g_stp2.PositionData.stLeftFootPosition.x = l_fb_stride_length;   g_stp2.PositionData.stLeftFootPosition.y = y_pos + l_rl_stride_length;    g_stp2.PositionData.stLeftFootPosition.z = -l_fb_stride_length*tan(l_foot_pitch);
		g_stp2.PositionData.stLeftFootPosition.roll = l_foot_roll;       g_stp2.PositionData.stLeftFootPosition.pitch = l_foot_pitch;              g_stp2.PositionData.stLeftFootPosition.yaw = l_foot_yaw;

		g_stp2.TimeData.bWalkingState = InWalking;
		g_stp2.TimeData.dAbsStepTime = l_se_time + l_period*2.0;         g_stp2.TimeData.dDSPratio = l_dsp;
		g_stp2.TimeData.sigmoid_ratio_x = 1.0;         g_stp2.TimeData.sigmoid_ratio_y = 1.0;          g_stp2.TimeData.sigmoid_ratio_z = 1.0;
		g_stp2.TimeData.sigmoid_ratio_roll = 1.0;      g_stp2.TimeData.sigmoid_ratio_pitch = 1.0;      g_stp2.TimeData.sigmoid_ratio_yaw = 1.0;
		g_stp2.TimeData.sigmoid_distortion_x = 1.0;    g_stp2.TimeData.sigmoid_distortion_y = 1.0;     g_stp2.TimeData.sigmoid_distortion_z = 1.0;
		g_stp2.TimeData.sigmoid_distortion_roll = 1.0; g_stp2.TimeData.sigmoid_distortion_pitch = 1.0; g_stp2.TimeData.sigmoid_distortion_yaw = 1.0;
	}
	else
	{
		g_stp1.PositionData.bMovingFoot = LFootMove;        g_stp1.PositionData.dFootHeight = l_foot_height;              g_stp1.PositionData.dZ_Swap_Amplitude = 10.0;
		g_stp1.PositionData.dShoulderSwingGain = 0.05;      g_stp1.PositionData.dElbowSwingGain = 0.1;

		g_stp1.PositionData.stRightFootPosition.x = 0.0;            g_stp1.PositionData.stRightFootPosition.y = -y_pos;           g_stp1.PositionData.stRightFootPosition.z = 0.0;
		g_stp1.PositionData.stRightFootPosition.roll = l_foot_roll; g_stp1.PositionData.stRightFootPosition.pitch = l_foot_pitch; g_stp1.PositionData.stRightFootPosition.yaw = l_foot_yaw;

		g_stp1.PositionData.stBodyPosition.z = l_body_height - 0.5*l_fb_stride_length*tan(l_foot_pitch);
		g_stp1.PositionData.stBodyPosition.roll = 0.0;      g_stp1.PositionData.stBodyPosition.pitch = 0.0;      g_stp1.PositionData.stBodyPosition.yaw = 0.0;

		g_stp1.PositionData.stLeftFootPosition.x = l_fb_stride_length; g_stp1.PositionData.stLeftFootPosition.y = y_pos + l_rl_stride_length; g_stp1.PositionData.stLeftFootPosition.z = -l_fb_stride_length*tan(l_foot_pitch);
		g_stp1.PositionData.stLeftFootPosition.roll = l_foot_roll;     g_stp1.PositionData.stLeftFootPosition.pitch = l_foot_pitch;           g_stp1.PositionData.stLeftFootPosition.yaw = l_foot_yaw;

		g_stp1.TimeData.bWalkingState = InWalking;
		g_stp1.TimeData.dAbsStepTime = l_se_time + l_period*1.0;         g_stp1.TimeData.dDSPratio = l_dsp;
		g_stp1.TimeData.sigmoid_ratio_x = 1.0;         g_stp1.TimeData.sigmoid_ratio_y = 1.0;    	   g_stp1.TimeData.sigmoid_ratio_z = 1.0;
		g_stp1.TimeData.sigmoid_ratio_roll = 1.0;      g_stp1.TimeData.sigmoid_ratio_pitch = 1.0; 	   g_stp1.TimeData.sigmoid_ratio_yaw = 1.0;
		g_stp1.TimeData.sigmoid_distortion_x = 1.0;    g_stp1.TimeData.sigmoid_distortion_y = 1.0;     g_stp1.TimeData.sigmoid_distortion_z = 1.0;
		g_stp1.TimeData.sigmoid_distortion_roll = 1.0; g_stp1.TimeData.sigmoid_distortion_pitch = 1.0; g_stp1.TimeData.sigmoid_distortion_yaw = 1.0;



		g_stp2.PositionData.bMovingFoot = RFootMove;        g_stp2.PositionData.dFootHeight = l_foot_height;              g_stp2.PositionData.dZ_Swap_Amplitude = 10.0;
		g_stp2.PositionData.dShoulderSwingGain = 0.05;      g_stp2.PositionData.dElbowSwingGain = 0.1;

		g_stp2.PositionData.stRightFootPosition.x = l_fb_stride_length;  g_stp2.PositionData.stRightFootPosition.y = -y_pos + l_rl_stride_length; g_stp2.PositionData.stRightFootPosition.z = -l_fb_stride_length*tan(l_foot_pitch);
		g_stp2.PositionData.stRightFootPosition.roll = l_foot_roll;      g_stp2.PositionData.stRightFootPosition.pitch = l_foot_pitch;            g_stp2.PositionData.stRightFootPosition.yaw = l_foot_yaw;

		g_stp2.PositionData.stBodyPosition.z = l_body_height - l_fb_stride_length*tan(l_foot_pitch);
		g_stp2.PositionData.stBodyPosition.roll = 0.0;    g_stp2.PositionData.stBodyPosition.pitch = 0.0;      g_stp2.PositionData.stBodyPosition.yaw = 0.0;

		g_stp2.PositionData.stLeftFootPosition.x = l_fb_stride_length;   g_stp2.PositionData.stLeftFootPosition.y = y_pos + l_rl_stride_length;   g_stp2.PositionData.stLeftFootPosition.z = -l_fb_stride_length*tan(l_foot_pitch);
		g_stp2.PositionData.stLeftFootPosition.roll = l_foot_roll;       g_stp2.PositionData.stLeftFootPosition.pitch = l_foot_pitch;             g_stp2.PositionData.stLeftFootPosition.yaw = l_foot_yaw;

		g_stp2.TimeData.bWalkingState = InWalking;
		g_stp2.TimeData.dAbsStepTime = l_se_time + l_period*2.0;  g_stp2.TimeData.dDSPratio = l_dsp;
		g_stp2.TimeData.sigmoid_ratio_x = 1.0;                  g_stp2.TimeData.sigmoid_ratio_y = 1.0;          g_stp2.TimeData.sigmoid_ratio_z = 1.0;
		g_stp2.TimeData.sigmoid_ratio_roll = 1.0;               g_stp2.TimeData.sigmoid_ratio_pitch = 1.0;      g_stp2.TimeData.sigmoid_ratio_yaw = 1.0;
		g_stp2.TimeData.sigmoid_distortion_x = 1.0;             g_stp2.TimeData.sigmoid_distortion_y = 1.0;     g_stp2.TimeData.sigmoid_distortion_z = 1.0;
		g_stp2.TimeData.sigmoid_distortion_roll = 1.0;          g_stp2.TimeData.sigmoid_distortion_pitch = 1.0; g_stp2.TimeData.sigmoid_distortion_yaw = 1.0;
	}

	g_stp3.PositionData.bMovingFoot = NFootMove;        g_stp3.PositionData.dFootHeight = 0.0;              g_stp3.PositionData.dZ_Swap_Amplitude = 0.0;
	g_stp3.PositionData.dShoulderSwingGain = 0.05;      g_stp3.PositionData.dElbowSwingGain = 0.1;
	g_stp3.PositionData.stRightFootPosition.x = l_fb_stride_length; g_stp3.PositionData.stRightFootPosition.y = -y_pos + l_rl_stride_length; g_stp3.PositionData.stRightFootPosition.z = - l_fb_stride_length*tan(l_foot_pitch);
	g_stp3.PositionData.stRightFootPosition.roll = l_foot_roll;     g_stp3.PositionData.stRightFootPosition.pitch = l_foot_pitch;            g_stp3.PositionData.stRightFootPosition.yaw = l_foot_yaw;

	g_stp3.PositionData.stBodyPosition.z = l_body_height - l_fb_stride_length*tan(l_foot_pitch);
	g_stp3.PositionData.stBodyPosition.roll = 0.0;      g_stp3.PositionData.stBodyPosition.pitch = 0.0;      g_stp3.PositionData.stBodyPosition.yaw = 0.0;

	g_stp3.PositionData.stLeftFootPosition.x = l_fb_stride_length;  g_stp3.PositionData.stLeftFootPosition.y = y_pos + l_rl_stride_length; g_stp3.PositionData.stLeftFootPosition.z = - l_fb_stride_length*tan(l_foot_pitch);
	g_stp3.PositionData.stLeftFootPosition.roll = l_foot_roll;      g_stp3.PositionData.stLeftFootPosition.pitch = l_foot_pitch;           g_stp3.PositionData.stLeftFootPosition.yaw = l_foot_yaw;

	g_stp3.TimeData.bWalkingState = InWalking;
	g_stp3.TimeData.dAbsStepTime = l_se_time*2.0 + l_period*2.0; g_stp3.TimeData.dDSPratio = l_dsp;
	g_stp3.TimeData.sigmoid_ratio_x = 1.0;                       g_stp3.TimeData.sigmoid_ratio_y = 1.0;          g_stp3.TimeData.sigmoid_ratio_z = 1.0;
	g_stp3.TimeData.sigmoid_ratio_roll = 1.0;                    g_stp3.TimeData.sigmoid_ratio_pitch = 1.0;      g_stp3.TimeData.sigmoid_ratio_yaw = 1.0;
	g_stp3.TimeData.sigmoid_distortion_x = 1.0;                  g_stp3.TimeData.sigmoid_distortion_y = 1.0;     g_stp3.TimeData.sigmoid_distortion_z = 1.0;
	g_stp3.TimeData.sigmoid_distortion_roll = 1.0;               g_stp3.TimeData.sigmoid_distortion_pitch = 1.0; g_stp3.TimeData.sigmoid_distortion_yaw = 1.0;

	RecursiveWalking::GetInstance()->SetRefZMPDecisionParameter(0.0, 0.0, g_y_zmp_convergence);
}

void UpdateRotateStepData(int first_moving_foot, double stride_degree)
{
	double y_pos = 105.0;
	double l_stride_radian = stride_degree*ANG2RAD;
	double l_fb_stride_length = y_pos*sin(l_stride_radian);
	double l_rl_stride_length = y_pos*cos(l_stride_radian);
	double l_rl_foot_heading = l_stride_radian;

	double l_dsp = g_dsp;
	double l_period = g_period;
	double l_se_time = g_se_time;
	double l_foot_height = g_foot_height;

	double l_foot_roll  = g_foot_roll;
	double l_foot_pitch = g_foot_pitch;
	double l_foot_yaw   = g_foot_yaw;
	double l_body_height = g_body_height;

	g_stp0.PositionData.bMovingFoot = NFootMove;                g_stp0.PositionData.dFootHeight = 0.0;               g_stp0.PositionData.dZ_Swap_Amplitude = 0.0;
	g_stp0.PositionData.dShoulderSwingGain = 0.05;              g_stp0.PositionData.dElbowSwingGain = 0.1;
	g_stp0.PositionData.stRightFootPosition.x = 0.0;            g_stp0.PositionData.stRightFootPosition.y = -y_pos;           g_stp0.PositionData.stRightFootPosition.z = 0.0;
	g_stp0.PositionData.stRightFootPosition.roll = l_foot_roll; g_stp0.PositionData.stRightFootPosition.pitch = l_foot_pitch; g_stp0.PositionData.stRightFootPosition.yaw = l_foot_yaw;

	g_stp0.PositionData.stBodyPosition.z = l_body_height;
	g_stp0.PositionData.stBodyPosition.roll = 0.0;              g_stp0.PositionData.stBodyPosition.pitch = 0.0;      g_stp0.PositionData.stBodyPosition.yaw = 0.0;

	g_stp0.PositionData.stLeftFootPosition.x = 0.0;             g_stp0.PositionData.stLeftFootPosition.y = y_pos;             g_stp0.PositionData.stLeftFootPosition.z = 0.0;
	g_stp0.PositionData.stLeftFootPosition.roll = l_foot_roll;  g_stp0.PositionData.stLeftFootPosition.pitch = l_foot_pitch;  g_stp0.PositionData.stLeftFootPosition.yaw = l_foot_yaw;

	g_stp0.TimeData.bWalkingState = InWalking;
	g_stp0.TimeData.dAbsStepTime = l_se_time;      g_stp0.TimeData.dDSPratio = l_dsp;
	g_stp0.TimeData.sigmoid_ratio_x = 1.0;         g_stp0.TimeData.sigmoid_ratio_y = 1.0;          g_stp0.TimeData.sigmoid_ratio_z = 1.0;
	g_stp0.TimeData.sigmoid_ratio_roll = 1.0;      g_stp0.TimeData.sigmoid_ratio_pitch = 1.0;      g_stp0.TimeData.sigmoid_ratio_yaw = 1.0;
	g_stp0.TimeData.sigmoid_distortion_x = 1.0;    g_stp0.TimeData.sigmoid_distortion_y = 1.0;     g_stp0.TimeData.sigmoid_distortion_z = 1.0;
	g_stp0.TimeData.sigmoid_distortion_roll = 1.0; g_stp0.TimeData.sigmoid_distortion_pitch = 1.0; g_stp0.TimeData.sigmoid_distortion_yaw = 1.0;


	if(first_moving_foot == RFootMove) {
		g_stp1.PositionData.bMovingFoot = RFootMove;        g_stp1.PositionData.dFootHeight = l_foot_height;              g_stp1.PositionData.dZ_Swap_Amplitude = 10.0;
		g_stp1.PositionData.dShoulderSwingGain = 0.05;      g_stp1.PositionData.dElbowSwingGain = 0.1;

		g_stp1.PositionData.stRightFootPosition.x = l_fb_stride_length;  g_stp1.PositionData.stRightFootPosition.y = -l_rl_stride_length; g_stp1.PositionData.stRightFootPosition.z = 0.0;
		g_stp1.PositionData.stRightFootPosition.roll = l_foot_roll;      g_stp1.PositionData.stRightFootPosition.pitch = l_foot_pitch;    g_stp1.PositionData.stRightFootPosition.yaw = l_foot_yaw + l_stride_radian;

		g_stp1.PositionData.stBodyPosition.z = l_body_height;
		g_stp1.PositionData.stBodyPosition.roll = 0.0;      g_stp1.PositionData.stBodyPosition.pitch = 0.0;      g_stp1.PositionData.stBodyPosition.yaw = 0.0;

		g_stp1.PositionData.stLeftFootPosition.x = 0.0;                  g_stp1.PositionData.stLeftFootPosition.y = y_pos;                g_stp1.PositionData.stLeftFootPosition.z = 0.0;
		g_stp1.PositionData.stLeftFootPosition.roll = l_foot_roll;       g_stp1.PositionData.stLeftFootPosition.pitch = l_foot_pitch;     g_stp1.PositionData.stLeftFootPosition.yaw = l_foot_yaw;

		g_stp1.TimeData.bWalkingState = InWalking;
		g_stp1.TimeData.dAbsStepTime = l_se_time + l_period*1;         g_stp1.TimeData.dDSPratio = l_dsp;
		g_stp1.TimeData.sigmoid_ratio_x = 1.0;         g_stp1.TimeData.sigmoid_ratio_y = 1.0;    	   g_stp1.TimeData.sigmoid_ratio_z = 1.0;
		g_stp1.TimeData.sigmoid_ratio_roll = 1.0;      g_stp1.TimeData.sigmoid_ratio_pitch = 1.0; 	   g_stp1.TimeData.sigmoid_ratio_yaw = 1.0;
		g_stp1.TimeData.sigmoid_distortion_x = 1.0;    g_stp1.TimeData.sigmoid_distortion_y = 1.0;     g_stp1.TimeData.sigmoid_distortion_z = 1.0;
		g_stp1.TimeData.sigmoid_distortion_roll = 1.0; g_stp1.TimeData.sigmoid_distortion_pitch = 1.0; g_stp1.TimeData.sigmoid_distortion_yaw = 1.0;



		g_stp2.PositionData.bMovingFoot = LFootMove;        g_stp2.PositionData.dFootHeight = l_foot_height;              g_stp2.PositionData.dZ_Swap_Amplitude = 10.0;
		g_stp2.PositionData.dShoulderSwingGain = 0.05;      g_stp2.PositionData.dElbowSwingGain = 0.1;

		g_stp2.PositionData.stRightFootPosition.x = l_fb_stride_length;  g_stp2.PositionData.stRightFootPosition.y = -l_rl_stride_length; g_stp2.PositionData.stRightFootPosition.z = 0.0;
		g_stp2.PositionData.stRightFootPosition.roll = l_foot_roll;      g_stp2.PositionData.stRightFootPosition.pitch = l_foot_pitch;    g_stp2.PositionData.stRightFootPosition.yaw = l_foot_yaw + l_stride_radian;

		g_stp2.PositionData.stBodyPosition.z = l_body_height;
		g_stp2.PositionData.stBodyPosition.roll = 0.0;    g_stp2.PositionData.stBodyPosition.pitch = 0.0;      g_stp2.PositionData.stBodyPosition.yaw = l_stride_radian;

		g_stp2.PositionData.stLeftFootPosition.x = -l_fb_stride_length; g_stp2.PositionData.stLeftFootPosition.y = l_rl_stride_length;  g_stp2.PositionData.stLeftFootPosition.z = 0.0;
		g_stp2.PositionData.stLeftFootPosition.roll = l_foot_roll;      g_stp2.PositionData.stLeftFootPosition.pitch = l_foot_pitch;    g_stp2.PositionData.stLeftFootPosition.yaw = l_foot_yaw + l_stride_radian;

		g_stp2.TimeData.bWalkingState = InWalking;
		g_stp2.TimeData.dAbsStepTime = l_se_time + l_period*2;         g_stp2.TimeData.dDSPratio = l_dsp;
		g_stp2.TimeData.sigmoid_ratio_x = 1.0;         g_stp2.TimeData.sigmoid_ratio_y = 1.0;          g_stp2.TimeData.sigmoid_ratio_z = 1.0;
		g_stp2.TimeData.sigmoid_ratio_roll = 1.0;      g_stp2.TimeData.sigmoid_ratio_pitch = 1.0;      g_stp2.TimeData.sigmoid_ratio_yaw = 1.0;
		g_stp2.TimeData.sigmoid_distortion_x = 1.0;    g_stp2.TimeData.sigmoid_distortion_y = 1.0;     g_stp2.TimeData.sigmoid_distortion_z = 1.0;
		g_stp2.TimeData.sigmoid_distortion_roll = 1.0; g_stp2.TimeData.sigmoid_distortion_pitch = 1.0; g_stp2.TimeData.sigmoid_distortion_yaw = 1.0;
	}
	else
	{
		g_stp1.PositionData.bMovingFoot = LFootMove;        g_stp1.PositionData.dFootHeight = l_foot_height;              g_stp1.PositionData.dZ_Swap_Amplitude = 10.0;
		g_stp1.PositionData.dShoulderSwingGain = 0.05;      g_stp1.PositionData.dElbowSwingGain = 0.1;

		g_stp1.PositionData.stRightFootPosition.x = 0.0;            g_stp1.PositionData.stRightFootPosition.y = -95.0;            g_stp1.PositionData.stRightFootPosition.z = 0.0;
		g_stp1.PositionData.stRightFootPosition.roll = l_foot_roll; g_stp1.PositionData.stRightFootPosition.pitch = l_foot_pitch; g_stp1.PositionData.stRightFootPosition.yaw = l_foot_yaw;

		g_stp1.PositionData.stBodyPosition.z = l_body_height;
		g_stp1.PositionData.stBodyPosition.roll = 0.0;      g_stp1.PositionData.stBodyPosition.pitch = 0.0;      g_stp1.PositionData.stBodyPosition.yaw = 0.0;

		g_stp1.PositionData.stLeftFootPosition.x = -l_fb_stride_length; g_stp1.PositionData.stLeftFootPosition.y = l_rl_stride_length; g_stp1.PositionData.stLeftFootPosition.z = 0.0;
		g_stp1.PositionData.stLeftFootPosition.roll = l_foot_roll;      g_stp1.PositionData.stLeftFootPosition.pitch = l_foot_pitch;   g_stp1.PositionData.stLeftFootPosition.yaw = l_foot_yaw + l_stride_radian;

		g_stp1.TimeData.bWalkingState = InWalking;
		g_stp1.TimeData.dAbsStepTime = l_se_time + l_period*1;         g_stp1.TimeData.dDSPratio = l_dsp;
		g_stp1.TimeData.sigmoid_ratio_x = 1.0;         g_stp1.TimeData.sigmoid_ratio_y = 1.0;    	   g_stp1.TimeData.sigmoid_ratio_z = 1.0;
		g_stp1.TimeData.sigmoid_ratio_roll = 1.0;      g_stp1.TimeData.sigmoid_ratio_pitch = 1.0; 	   g_stp1.TimeData.sigmoid_ratio_yaw = 1.0;
		g_stp1.TimeData.sigmoid_distortion_x = 1.0;    g_stp1.TimeData.sigmoid_distortion_y = 1.0;     g_stp1.TimeData.sigmoid_distortion_z = 1.0;
		g_stp1.TimeData.sigmoid_distortion_roll = 1.0; g_stp1.TimeData.sigmoid_distortion_pitch = 1.0; g_stp1.TimeData.sigmoid_distortion_yaw = 1.0;



		g_stp2.PositionData.bMovingFoot = RFootMove;        g_stp2.PositionData.dFootHeight = l_foot_height;              g_stp2.PositionData.dZ_Swap_Amplitude = 10.0;
		g_stp2.PositionData.dShoulderSwingGain = 0.05;      g_stp2.PositionData.dElbowSwingGain = 0.1;

		g_stp2.PositionData.stRightFootPosition.x = l_fb_stride_length;  g_stp2.PositionData.stRightFootPosition.y = -l_rl_stride_length; g_stp2.PositionData.stRightFootPosition.z = 0.0;
		g_stp2.PositionData.stRightFootPosition.roll = l_foot_roll;      g_stp2.PositionData.stRightFootPosition.pitch = l_foot_pitch;    g_stp2.PositionData.stRightFootPosition.yaw = l_foot_yaw + l_stride_radian;

		g_stp2.PositionData.stBodyPosition.z = l_body_height;
		g_stp2.PositionData.stBodyPosition.roll = 0.0;    g_stp2.PositionData.stBodyPosition.pitch = 0.0;      g_stp2.PositionData.stBodyPosition.yaw = l_stride_radian;

		g_stp2.PositionData.stLeftFootPosition.x = -l_fb_stride_length; g_stp2.PositionData.stLeftFootPosition.y = l_rl_stride_length;  g_stp2.PositionData.stLeftFootPosition.z = 0.0;
		g_stp2.PositionData.stLeftFootPosition.roll = l_foot_roll;      g_stp2.PositionData.stLeftFootPosition.pitch = l_foot_pitch;    g_stp2.PositionData.stLeftFootPosition.yaw = l_foot_yaw + l_stride_radian;

		g_stp2.TimeData.bWalkingState = InWalking;
		g_stp2.TimeData.dAbsStepTime = l_se_time + l_period*2;         g_stp2.TimeData.dDSPratio = l_dsp;
		g_stp2.TimeData.sigmoid_ratio_x = 1.0;         g_stp2.TimeData.sigmoid_ratio_y = 1.0;          g_stp2.TimeData.sigmoid_ratio_z = 1.0;
		g_stp2.TimeData.sigmoid_ratio_roll = 1.0;      g_stp2.TimeData.sigmoid_ratio_pitch = 1.0;      g_stp2.TimeData.sigmoid_ratio_yaw = 1.0;
		g_stp2.TimeData.sigmoid_distortion_x = 1.0;    g_stp2.TimeData.sigmoid_distortion_y = 1.0;     g_stp2.TimeData.sigmoid_distortion_z = 1.0;
		g_stp2.TimeData.sigmoid_distortion_roll = 1.0; g_stp2.TimeData.sigmoid_distortion_pitch = 1.0; g_stp2.TimeData.sigmoid_distortion_yaw = 1.0;
	}

	g_stp3.PositionData.bMovingFoot = NFootMove;        g_stp3.PositionData.dFootHeight = 0.0;              g_stp3.PositionData.dZ_Swap_Amplitude = 0.0;
	g_stp3.PositionData.dShoulderSwingGain = 0.05;      g_stp3.PositionData.dElbowSwingGain = 0.1;
	g_stp3.PositionData.stRightFootPosition.x = l_fb_stride_length;  g_stp3.PositionData.stRightFootPosition.y = -l_rl_stride_length;   g_stp3.PositionData.stRightFootPosition.z = 0.0;
	g_stp3.PositionData.stRightFootPosition.roll = l_foot_roll;      g_stp3.PositionData.stRightFootPosition.pitch = l_foot_pitch;      g_stp3.PositionData.stRightFootPosition.yaw = l_foot_yaw + l_stride_radian;

	g_stp3.PositionData.stBodyPosition.z = l_body_height;
	g_stp3.PositionData.stBodyPosition.roll = 0.0;      g_stp3.PositionData.stBodyPosition.pitch = 0.0;      g_stp3.PositionData.stBodyPosition.yaw = l_stride_radian;

	g_stp3.PositionData.stLeftFootPosition.x = -l_fb_stride_length;   g_stp3.PositionData.stLeftFootPosition.y = l_rl_stride_length;  g_stp3.PositionData.stLeftFootPosition.z = 0.0;
	g_stp3.PositionData.stLeftFootPosition.roll = l_foot_roll;        g_stp3.PositionData.stLeftFootPosition.pitch = l_foot_pitch;    g_stp3.PositionData.stLeftFootPosition.yaw = l_foot_yaw + l_stride_radian;

	g_stp3.TimeData.bWalkingState = InWalking;
	g_stp3.TimeData.dAbsStepTime = l_se_time*2.0 + l_period*2.0; g_stp3.TimeData.dDSPratio = l_dsp;
	g_stp3.TimeData.sigmoid_ratio_x = 1.0;                       g_stp3.TimeData.sigmoid_ratio_y = 1.0;          g_stp3.TimeData.sigmoid_ratio_z = 1.0;
	g_stp3.TimeData.sigmoid_ratio_roll = 1.0;                    g_stp3.TimeData.sigmoid_ratio_pitch = 1.0;      g_stp3.TimeData.sigmoid_ratio_yaw = 1.0;
	g_stp3.TimeData.sigmoid_distortion_x = 1.0;                  g_stp3.TimeData.sigmoid_distortion_y = 1.0;     g_stp3.TimeData.sigmoid_distortion_z = 1.0;
	g_stp3.TimeData.sigmoid_distortion_roll = 1.0;               g_stp3.TimeData.sigmoid_distortion_pitch = 1.0; g_stp3.TimeData.sigmoid_distortion_yaw = 1.0;

	RecursiveWalking::GetInstance()->SetRefZMPDecisionParameter(0.0, 0.0, g_y_zmp_convergence);
}

StepData stp0, stp1, stp2, stp3, stp4;
void UpLadder(int first_moving_foot, double incline_angle_deg)
{

	  //we need to make a Initialize func
	  stp0.PositionData.bMovingFoot = NFootMove;        stp0.PositionData.dFootHeight = 0.0;               stp0.PositionData.dZ_Swap_Amplitude = 0.0;
	  stp0.PositionData.dShoulderSwingGain = 0.05;      stp0.PositionData.dElbowSwingGain = 0.1;
	  stp0.PositionData.stRightFootPosition.x = 0.0;    stp0.PositionData.stRightFootPosition.y = -95.0;   stp0.PositionData.stRightFootPosition.z = 0.0;
	  stp0.PositionData.stRightFootPosition.roll = 0.0; stp0.PositionData.stRightFootPosition.pitch = 0.0; stp0.PositionData.stRightFootPosition.yaw = 0.0;
	  stp0.PositionData.stBodyPosition.z = 650.0;
	  stp0.PositionData.stBodyPosition.roll = 0.0;      stp0.PositionData.stBodyPosition.pitch = 0.0;      stp0.PositionData.stBodyPosition.yaw = 0.0;
	  stp0.PositionData.stLeftFootPosition.x = 0.0;     stp0.PositionData.stLeftFootPosition.y = 95.0;     stp0.PositionData.stLeftFootPosition.z = 0.0;
	  stp0.PositionData.stLeftFootPosition.roll = 0.0;  stp0.PositionData.stLeftFootPosition.pitch = 0.0;  stp0.PositionData.stLeftFootPosition.yaw = 0.0;

	  stp0.TimeData.bWalkingState = InWalkingStarting;
	  stp0.TimeData.dAbsStepTime = 3000.0;    stp0.TimeData.dDSPratio = 0.2;
	  stp0.TimeData.sigmoid_ratio_x = 1.0;    stp0.TimeData.sigmoid_ratio_y = 1.0;     stp0.TimeData.sigmoid_ratio_z = 1.0;
	  stp0.TimeData.sigmoid_ratio_roll = 1.0; stp0.TimeData.sigmoid_ratio_pitch = 1.0; stp0.TimeData.sigmoid_ratio_yaw = 1.0;
	  stp0.TimeData.sigmoid_distortion_x = 1.0;    stp0.TimeData.sigmoid_distortion_y = 1.0;     stp0.TimeData.sigmoid_distortion_z = 1.0;
	  stp0.TimeData.sigmoid_distortion_roll = 1.0; stp0.TimeData.sigmoid_distortion_pitch = 1.0; stp0.TimeData.sigmoid_distortion_yaw = 1.0;



	  stp1.PositionData.bMovingFoot = RFootMove;        stp1.PositionData.dFootHeight = 80.0;               stp1.PositionData.dZ_Swap_Amplitude = 10.0;
	  stp1.PositionData.dShoulderSwingGain = 0.05;      stp1.PositionData.dElbowSwingGain = 0.1;
	  stp1.PositionData.stRightFootPosition.x = 140.0;  stp1.PositionData.stRightFootPosition.y = -95.0;   stp1.PositionData.stRightFootPosition.z = 310.0;
	  stp1.PositionData.stRightFootPosition.roll = 0.0; stp1.PositionData.stRightFootPosition.pitch = 0.0; stp1.PositionData.stRightFootPosition.yaw = 0.0;
	  stp1.PositionData.stBodyPosition.z = 650.0;
	  stp1.PositionData.stBodyPosition.roll = 0.0;      stp1.PositionData.stBodyPosition.pitch = 0.0;      stp1.PositionData.stBodyPosition.yaw = 0.0;
	  stp1.PositionData.stLeftFootPosition.x = 0.0;     stp1.PositionData.stLeftFootPosition.y = 95.0;     stp1.PositionData.stLeftFootPosition.z = 0.0;
	  stp1.PositionData.stLeftFootPosition.roll = 0.0;  stp1.PositionData.stLeftFootPosition.pitch = 0.0;  stp1.PositionData.stLeftFootPosition.yaw = 0.0;

	  stp1.TimeData.bWalkingState = InWalking;
	  stp1.TimeData.dAbsStepTime = 3850.0;    stp1.TimeData.dDSPratio = 0.2;
	  stp1.TimeData.sigmoid_ratio_x = 1.0;    stp1.TimeData.sigmoid_ratio_y = 1.0;     stp1.TimeData.sigmoid_ratio_z = 1.0;
	  stp1.TimeData.sigmoid_ratio_roll = 1.0; stp1.TimeData.sigmoid_ratio_pitch = 1.0; stp1.TimeData.sigmoid_ratio_yaw = 1.0;
	  stp1.TimeData.sigmoid_distortion_x = 1.0;    stp1.TimeData.sigmoid_distortion_y = 1.0;     stp1.TimeData.sigmoid_distortion_z = 1.0;
	  stp1.TimeData.sigmoid_distortion_roll = 1.0; stp1.TimeData.sigmoid_distortion_pitch = 1.0; stp1.TimeData.sigmoid_distortion_yaw = 1.0;



	  stp2.PositionData.bMovingFoot = NFootMove;        stp2.PositionData.dFootHeight = 80.0;               stp2.PositionData.dZ_Swap_Amplitude = 10.0;
	  stp2.PositionData.dShoulderSwingGain = 0.05;      stp2.PositionData.dElbowSwingGain = 0.1;
	  stp2.PositionData.stRightFootPosition.x = 140.0;  stp2.PositionData.stRightFootPosition.y = -95.0;   stp2.PositionData.stRightFootPosition.z = 310.0;
	  stp2.PositionData.stRightFootPosition.roll = 0.0; stp2.PositionData.stRightFootPosition.pitch = 0.0; stp2.PositionData.stRightFootPosition.yaw = 0.0;
	  stp2.PositionData.stBodyPosition.z = 650.0;
	  stp2.PositionData.stBodyPosition.roll = 0.0;      stp2.PositionData.stBodyPosition.pitch = 0.0;      stp2.PositionData.stBodyPosition.yaw = 0.0;
	  stp2.PositionData.stLeftFootPosition.x = 280.0;     stp2.PositionData.stLeftFootPosition.y = 95.0;   stp2.PositionData.stLeftFootPosition.z = 0.0;
	  stp2.PositionData.stLeftFootPosition.roll = 0.0;  stp2.PositionData.stLeftFootPosition.pitch = 0.0;  stp2.PositionData.stLeftFootPosition.yaw = 0.0;

	  stp2.TimeData.bWalkingState = InWalking;
	  stp2.TimeData.dAbsStepTime = 4700.0;    stp2.TimeData.dDSPratio = 0.2;
	  stp2.TimeData.sigmoid_ratio_x = 1.0;    stp2.TimeData.sigmoid_ratio_y = 1.0;     stp2.TimeData.sigmoid_ratio_z = 1.0;
	  stp2.TimeData.sigmoid_ratio_roll = 1.0; stp2.TimeData.sigmoid_ratio_pitch = 1.0; stp2.TimeData.sigmoid_ratio_yaw = 1.0;
	  stp2.TimeData.sigmoid_distortion_x = 1.0;    stp2.TimeData.sigmoid_distortion_y = 1.0;     stp2.TimeData.sigmoid_distortion_z = 1.0;
	  stp2.TimeData.sigmoid_distortion_roll = 1.0; stp2.TimeData.sigmoid_distortion_pitch = 1.0; stp2.TimeData.sigmoid_distortion_yaw = 1.0;



	  stp3.PositionData.bMovingFoot = RFootMove;        stp3.PositionData.dFootHeight = 80.0;               stp3.PositionData.dZ_Swap_Amplitude = 10.0;
	  stp3.PositionData.dShoulderSwingGain = 0.05;      stp3.PositionData.dElbowSwingGain = 0.1;
	  stp3.PositionData.stRightFootPosition.x = 280.0;  stp3.PositionData.stRightFootPosition.y = -95.0;   stp3.PositionData.stRightFootPosition.z = 0.0;
	  stp3.PositionData.stRightFootPosition.roll = 0.0; stp3.PositionData.stRightFootPosition.pitch = 0.0; stp3.PositionData.stRightFootPosition.yaw = 0.0;
	  stp3.PositionData.stBodyPosition.z = 670.0;
	  stp3.PositionData.stBodyPosition.roll = 0.0;      stp3.PositionData.stBodyPosition.pitch = 0.0;      stp3.PositionData.stBodyPosition.yaw = 0.0;
	  stp3.PositionData.stLeftFootPosition.x = 280.0;     stp3.PositionData.stLeftFootPosition.y = 95.0;   stp3.PositionData.stLeftFootPosition.z = 0.0;
	  stp3.PositionData.stLeftFootPosition.roll = 0.0;  stp3.PositionData.stLeftFootPosition.pitch = 0.0;  stp3.PositionData.stLeftFootPosition.yaw = 0.0;

	  stp3.TimeData.bWalkingState = InWalking;
	  stp3.TimeData.dAbsStepTime = 5500.0;    stp3.TimeData.dDSPratio = 0.2;
	  stp3.TimeData.sigmoid_ratio_x = 1.0;    stp3.TimeData.sigmoid_ratio_y = 1.0;     stp3.TimeData.sigmoid_ratio_z = 1.0;
	  stp3.TimeData.sigmoid_ratio_roll = 1.0; stp3.TimeData.sigmoid_ratio_pitch = 1.0; stp3.TimeData.sigmoid_ratio_yaw = 1.0;
	  stp3.TimeData.sigmoid_distortion_x = 1.0;    stp3.TimeData.sigmoid_distortion_y = 1.0;     stp3.TimeData.sigmoid_distortion_z = 1.0;
	  stp3.TimeData.sigmoid_distortion_roll = 1.0; stp3.TimeData.sigmoid_distortion_pitch = 1.0; stp3.TimeData.sigmoid_distortion_yaw = 1.0;


	  stp4.PositionData.bMovingFoot = NFootMove;        stp4.PositionData.dFootHeight = 0.0;               stp4.PositionData.dZ_Swap_Amplitude = 0.0;
	  stp4.PositionData.dShoulderSwingGain = 0.05;      stp4.PositionData.dElbowSwingGain = 0.1;
	  stp4.PositionData.stRightFootPosition.x = 280.0;  stp4.PositionData.stRightFootPosition.y = -95.0;   stp4.PositionData.stRightFootPosition.z = 0.0;
	  stp4.PositionData.stRightFootPosition.roll = 0.0; stp4.PositionData.stRightFootPosition.pitch = 0.0; stp4.PositionData.stRightFootPosition.yaw = 0.0;
	  stp4.PositionData.stBodyPosition.z = 690.0;
	  stp4.PositionData.stBodyPosition.roll = 0.0;      stp4.PositionData.stBodyPosition.pitch = 0.0;      stp4.PositionData.stBodyPosition.yaw = 0.0;
	  stp4.PositionData.stLeftFootPosition.x = 280.0;   stp4.PositionData.stLeftFootPosition.y = 95.0;     stp4.PositionData.stLeftFootPosition.z = 0.0;
	  stp4.PositionData.stLeftFootPosition.roll = 0.0;  stp4.PositionData.stLeftFootPosition.pitch = 0.0;  stp4.PositionData.stLeftFootPosition.yaw = 0.0;

	  stp4.TimeData.bWalkingState = InWalking;
	  stp4.TimeData.dAbsStepTime = 8500.0;    stp4.TimeData.dDSPratio = 0.2;
	  stp4.TimeData.sigmoid_ratio_x = 1.0;    stp4.TimeData.sigmoid_ratio_y = 1.0;     stp4.TimeData.sigmoid_ratio_z = 1.0;
	  stp4.TimeData.sigmoid_ratio_roll = 1.0; stp4.TimeData.sigmoid_ratio_pitch = 1.0; stp4.TimeData.sigmoid_ratio_yaw = 1.0;
	  stp4.TimeData.sigmoid_distortion_x = 1.0;    stp4.TimeData.sigmoid_distortion_y = 1.0;     stp4.TimeData.sigmoid_distortion_z = 1.0;
	  stp4.TimeData.sigmoid_distortion_roll = 1.0; stp4.TimeData.sigmoid_distortion_pitch = 1.0; stp4.TimeData.sigmoid_distortion_yaw = 1.0;
}

int _getch()
{
	struct termios oldt, newt;
	int ch;
	tcgetattr( STDIN_FILENO, &oldt );
	newt = oldt;
	newt.c_lflag &= ~(ICANON | ECHO);
	tcsetattr( STDIN_FILENO, TCSANOW, &newt );
	ch = getchar();
	tcsetattr( STDIN_FILENO, TCSANOW, &oldt );
	return ch;
}

int kbhit(void)
{
	struct termios oldt, newt;
	int ch;
	int oldf;

	tcgetattr(STDIN_FILENO, &oldt);
	newt = oldt;
	newt.c_lflag &= ~(ICANON | ECHO);
	tcsetattr(STDIN_FILENO, TCSANOW, &newt);
	oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
	fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

	ch = getchar();

	tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
	fcntl(STDIN_FILENO, F_SETFL, oldf);

	if(ch != EOF)
	{
		ungetc(ch, stdin);
		return 1;
	}

	return 0;
}

void Help()
{
	fprintf(stderr, "\n");
	fprintf(stderr, "COMMAND: \n\n" \
			" forward       : forward walking\n"
			" climbblock    : block climbing \n" \
			"release        : hand release \n"\
			"grab           : hand grab \n"\
			"h              : hand hello\n"\
			"initpose       :  \n"\
			"lookvalve      :   \n"\
			"fastenvalve     :  \n"\
			" exit          : Exit this program \n" \
	);

	//			" c             : walking down the block \n" \
	//			" d             : ??? \n" \//
	fprintf(stderr, "\n");
}

// Print error bit of status packet
void PrintErrorCode(int ErrorCode)
{
	printf("ErrorCode : %d (0x%X)\n", ErrorCode, ErrorCode);
	if(ErrorCode & ERRBIT_VOLTAGE)
		printf("Input voltage error!\n");

	if(ErrorCode & ERRBIT_ANGLE)
		printf("Angle limit error!\n");

	if(ErrorCode & ERRBIT_OVERHEAT)
		printf("Overheat error!\n");

	if(ErrorCode & ERRBIT_RANGE)
		printf("Out of range error!\n");

	if(ErrorCode & ERRBIT_CHECKSUM)
		printf("Checksum error!\n");

	if(ErrorCode & ERRBIT_OVERLOAD)
		printf("Overload error!\n");

	if(ErrorCode & ERRBIT_INSTRUCTION)
		printf("Instruction code error!\n");
}

void SetTheMotionEnableList(void)
{
	for(unsigned int jointIndex = 0; jointIndex < 35; jointIndex++)
	{
		int id = jointIndex;

		//not be fixed code
		if(id >= 15 && id <=26)
		{
			MotionStatus::m_EnableList[id-1].uID = "RecursiveWalking";
		}
		if(id == 1 || id ==2 || id == 7|| id ==8)
			MotionStatus::m_EnableList[id-1].uID = "RecursiveWalking";
	}

	MotionStatus::m_EnableList[27 - 1].uID = "Action";
	MotionStatus::m_EnableList[28 - 1].uID = "Action";
	MotionStatus::m_EnableList[29 - 1].uID = "Action";
	MotionStatus::m_EnableList[30 - 1].uID = "Action";
}

const int StepForward = 0;
const int StepBackrward = 1;
const int StepRight = 2;
const int StepLeft = 3;

const int StepRightRotate = 0;
const int StepLeftRotate = 1;

const int first_flat = 0;
const int on_the_ramp = 1;
const int on_the_ramp1 = 1;

void UpdateTranslateParamter(int phase, int kind_of_step);
void UpdateRotateParamter(int phase, int kind_of_step);

int main(int argc, char *argv[])
{
	fprintf(stderr, "\n***********************************************************************\n");
	fprintf(stderr,   "*                            Demo for DRC                             *\n");
	fprintf(stderr,   "***********************************************************************\n\n");


	////////////////////////////////////////////// Motion Manager Initializing //////////////////////////////////////////////
	if(MotionManager::GetInstance()->Initialize() == false)
	{
		printf("Fail to initialize Motion Manager!\n");
		return 0;
	}

	minIni* ini = new minIni("config.ini");
	MotionManager::GetInstance()->LoadINISettings(ini);

	////////////////////////////////////////////////////// Speed Down //////////////////////////////////////////////////////
	for(unsigned int index = 0; index < MotionStatus::m_CurrentJoints.size(); index++)
	{
		int id = MotionStatus::m_CurrentJoints[index].m_ID;

		int err;
		MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteByte(id, PRO54::P_TORQUE_ENABLE, 1, &err);
		MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteDWord(id, PRO54::P_GOAL_ACCELATION_LL, 4, &err);
		MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteDWord(id, PRO54::P_GOAL_VELOCITY_LL, 2000, &err);
		printf("id : %d  ", id);
		PrintErrorCode(err);
		usleep(1000);
	}

	//////////////////////////////////// Ins Initializing and Start Update of Initialize ////////////////////////////////////
	Ins *ins = new Ins();
	if(ins->Connect("ttyACM0", 921600) !=MIP_INTERFACE_OK)
	{
		printf("fail to connect\n");
		return 0;
	}
	printf( "\n===== start initializing ins  =====\n\n");
	if(ins->Initialize() != MIP_INTERFACE_OK)
	{
		printf("fail to init\n");
		return 0;
	}
	printf( "\n===== set enalble data callback =====\n\n");

	if(ins->SetEnableAHRSDataCallBack() != MIP_INTERFACE_OK)
	{
		printf("fail to init\n");
		return 0;
	}
	ins->StartSensingDataUpdate();


	printf("Press the any button to move first pose!\n");
	_getch();


	int dir_output[16];
	double InitAngle[16];

	//for thor
	dir_output[0] = -1; dir_output[1] = -1; dir_output[2] = -1; dir_output[3] = -1; dir_output[4] =  1; dir_output[5] = 1;
	dir_output[6] = -1; dir_output[7] = -1; dir_output[8] =  1; dir_output[9] =  1; dir_output[10]= -1; dir_output[11] = 1;
	dir_output[12] = -1; dir_output[13] = 1;  dir_output[14] = -1; dir_output[15] =  1;
	InitAngle[0]  =   0.0;  InitAngle[1]  =  0.0;  InitAngle[2]  =  5.7106;  InitAngle[3] =  33.5788; InitAngle[4]  = -5.7106; InitAngle[5]  = 0.0;
	InitAngle[6]  =   0.0;  InitAngle[7]  =  0.0;  InitAngle[8]  = -5.7106;  InitAngle[9] = -33.5788; InitAngle[10] =  5.7106; InitAngle[11] = 0.0;
	InitAngle[12] = -45.0,  InitAngle[13] = 45.0;  InitAngle[14] =  45.0;    InitAngle[15] =  -45.0;

	double angle[16];
	int outValue[16];

	matd GtoCOB = GetTransformMatrix(0, 0, 650, 0, 0, 0 );
	matd GtoRF = GetTransformMatrix(0, -105, 0, 0, 0, 0);
	matd GtoLF = GetTransformMatrix(0,  105, 0, 0, 0, 0);

	matd RHtoCOB = GetTranslationMatrix(0,  Kinematics::LEG_SIDE_OFFSET*0.5, 0);
	matd LHtoCOB = GetTranslationMatrix(0, -Kinematics::LEG_SIDE_OFFSET*0.5, 0);

	matd COBtoG = GetTransformMatrixInverse(GtoCOB);
	matd RHtoRF = RHtoCOB*COBtoG*GtoRF;
	matd LHtoLF = LHtoCOB*COBtoG*GtoLF;

	Pose3D epr, epl;

	epr = GetPose3DfromTransformMatrix(RHtoRF);
	epl = GetPose3DfromTransformMatrix(LHtoLF);

	if(RecursiveWalking::GetInstance()->computeIK(&angle[0], epr.x, epr.y, epr.z+Kinematics::LEG_LENGTH, epr.roll, epr.pitch, epr.yaw) == false) {
		printf("IKsolve failed\n");
		return 0;
	}

	if(RecursiveWalking::GetInstance()->computeIK(&angle[6], epl.x, epl.y, epl.z+Kinematics::LEG_LENGTH, epl.roll, epl.pitch, epl.yaw) == false) {
		printf("IKsolve failed\n");
		return 0;
	}

	for(int idx = 0; idx < 6; idx++)	{
		angle[idx] = (double)dir_output[idx]*angle[idx]*180.0/PI + InitAngle[idx];
		angle[idx+6] = (double)dir_output[idx+6]*angle[idx+6]*180.0/PI + InitAngle[idx+6];
	}


	for(int idx = 0; idx < 16; idx++)	{
		outValue[idx] = 251000.0*(angle[idx])/180.0;
	}

	outValue[2] -= (double)dir_output[2] * 8.0 * 251000.0/180.0;
	outValue[8] -= (double)dir_output[8] * 8.0 * 251000.0/180.0;

	for(unsigned int index = 0; index < MotionStatus::m_CurrentJoints.size(); index++) {
		int id = MotionStatus::m_CurrentJoints[index].m_ID;

		int err;
		if(id >= 15 && id <= 26) {
			MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteWord(id, PRO54::P_VELOCITY_I_GAIN_L, 0, &err);

		}
		printf("id : %d  ", id);
		PrintErrorCode(err);
		usleep(1000);
	}

	for(unsigned int index = 0; index < MotionStatus::m_CurrentJoints.size(); index++)
	{
		int id = MotionStatus::m_CurrentJoints[index].m_ID;

		int err = 0;
		if( id == 1)
			MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteDWord(id, PRO54::P_GOAL_POSITION_LL, -62750, &err);
		else if(id == 2)
			MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteDWord(id, PRO54::P_GOAL_POSITION_LL, 62750, &err);
		else if(id == 3)
			MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteDWord(id, PRO54::P_GOAL_POSITION_LL, -109520, &err);
		else if(id == 4)
			MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteDWord(id, PRO54::P_GOAL_POSITION_LL, 109520, &err);
		else if(id == 5)
			MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteDWord(id, PRO54::P_GOAL_POSITION_LL, 125500, &err);
		else if(id == 6)
			MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteDWord(id, PRO54::P_GOAL_POSITION_LL, -125500, &err);
		else if(id == 7)
			MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteDWord(id, PRO54::P_GOAL_POSITION_LL, 62750, &err);
		else if(id == 8)
			MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteDWord(id, PRO54::P_GOAL_POSITION_LL, -62750, &err);
		else if(id == 9)
			MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteDWord(id, PRO54::P_GOAL_POSITION_LL, -75000, &err);
		else if(id == 10)
			MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteDWord(id, PRO54::P_GOAL_POSITION_LL,  75000, &err);


		else if((id == 27) || (id == 28) || (id == 29))
			MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteDWord(id, PRO54::P_GOAL_POSITION_LL,  0, &err);
		else if((id == 30))
			MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteDWord(id, PRO54::P_GOAL_POSITION_LL,  60000, &err);
		else if(id == 30)
			MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteDWord(id, PRO54::P_GOAL_POSITION_LL,  0, &err);
		else if(id == 12)
			MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteDWord(id, PRO54::P_GOAL_POSITION_LL,  0, &err);
		else if(id == 13)
			MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteDWord(id, PRO54::P_GOAL_POSITION_LL,  0, &err);
		else if(id == 14)
			MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteDWord(id, PRO54::P_GOAL_POSITION_LL,  0, &err);


		else if(id == 15)
			MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteDWord(id, PRO54::P_GOAL_POSITION_LL,  outValue[0] + MotionManager::GetInstance()->m_Offset[14], &err);
		else if(id == 17)
			MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteDWord(id, PRO54::P_GOAL_POSITION_LL,  outValue[1] + MotionManager::GetInstance()->m_Offset[16], &err);
		else if(id == 19)
			MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteDWord(id, PRO54::P_GOAL_POSITION_LL,  outValue[2] + MotionManager::GetInstance()->m_Offset[18], &err);
		else if(id == 21)
			MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteDWord(id, PRO54::P_GOAL_POSITION_LL,  outValue[3] + MotionManager::GetInstance()->m_Offset[20], &err);
		else if(id == 23)
			MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteDWord(id, PRO54::P_GOAL_POSITION_LL,  outValue[4] + MotionManager::GetInstance()->m_Offset[22], &err);
		else if(id == 25)
			MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteDWord(id, PRO54::P_GOAL_POSITION_LL,  outValue[5] + MotionManager::GetInstance()->m_Offset[24], &err);

		else if(id == 16)
			MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteDWord(id, PRO54::P_GOAL_POSITION_LL,  outValue[6] + MotionManager::GetInstance()->m_Offset[15], &err);
		else if(id == 18)
			MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteDWord(id, PRO54::P_GOAL_POSITION_LL,  outValue[7] + MotionManager::GetInstance()->m_Offset[17], &err);
		else if(id == 20)
			MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteDWord(id, PRO54::P_GOAL_POSITION_LL,  outValue[8] + MotionManager::GetInstance()->m_Offset[19], &err);
		else if(id == 22)
			MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteDWord(id, PRO54::P_GOAL_POSITION_LL,  outValue[9] + MotionManager::GetInstance()->m_Offset[21], &err);
		else if(id == 24)
			MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteDWord(id, PRO54::P_GOAL_POSITION_LL,  outValue[10] + MotionManager::GetInstance()->m_Offset[23], &err);
		else if(id == 26)
			MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteDWord(id, PRO54::P_GOAL_POSITION_LL,  outValue[11] + MotionManager::GetInstance()->m_Offset[25], &err);

		usleep(1000);
	}

	usleep(7000000);
	printf("Press the any button. After the moving stop!\n");
	_getch();

	MotionManager::GetInstance()->Reinitialize();



	RecursiveWalking::GetInstance()->BALANCE_ENABLE = true;
	RecursiveWalking::GetInstance()->DEBUG_PRINT = false;

	RecursiveWalking::GetInstance()->HIP_PITCH_OFFSET = 8.0;//6.0
	RecursiveWalking::GetInstance()->Initialize();

	RecursiveWalking::GetInstance()->WALK_STABILIZER_GAIN_RATIO = 3.5;
	RecursiveWalking::GetInstance()->IMU_GYRO_GAIN_RATIO = 0.0;//7.31*0.01;
	RecursiveWalking::GetInstance()->FORCE_MOMENT_DISTRIBUTION_RATIO = 0.4;

	RecursiveWalking::GetInstance()->BALANCE_X_GAIN     = +20.30*0.625*(RecursiveWalking::GetInstance()->FORCE_MOMENT_DISTRIBUTION_RATIO)*RecursiveWalking::GetInstance()->WALK_STABILIZER_GAIN_RATIO;
	RecursiveWalking::GetInstance()->BALANCE_Y_GAIN     = -20.30*(RecursiveWalking::GetInstance()->FORCE_MOMENT_DISTRIBUTION_RATIO)*RecursiveWalking::GetInstance()->WALK_STABILIZER_GAIN_RATIO;
	RecursiveWalking::GetInstance()->BALANCE_PITCH_GAIN = -0.06*0.625*(1-RecursiveWalking::GetInstance()->FORCE_MOMENT_DISTRIBUTION_RATIO)*RecursiveWalking::GetInstance()->WALK_STABILIZER_GAIN_RATIO;
	RecursiveWalking::GetInstance()->BALANCE_ROLL_GAIN  = -0.10*(1-RecursiveWalking::GetInstance()->FORCE_MOMENT_DISTRIBUTION_RATIO)*RecursiveWalking::GetInstance()->WALK_STABILIZER_GAIN_RATIO;

	RecursiveWalking::GetInstance()->FOOT_LANDING_OFFSET_GAIN =   +1.0*0;
	RecursiveWalking::GetInstance()->FOOT_LANDING_DETECT_N = 50;

	RecursiveWalking::GetInstance()->SYSTEM_CONTROL_UNIT_TIME_SEC = MotionModule::TIME_UNIT / 1000.0;
	RecursiveWalking::GetInstance()->FOOT_LANDING_DETECTION_TIME_MAX_SEC = 10.0;

	RecursiveWalking::GetInstance()->FOOT_ROLL_ADJUSTMENT_ABS_MAX_RAD = 10.0*PI/180;
	RecursiveWalking::GetInstance()->FOOT_PITCH_ADJUSTMENT_ABS_MAX_RAD = 10.0*PI/180;

	RecursiveWalking::GetInstance()->COB_X_ADJUSTMENT_ABS_MAX_MM = 50;
	RecursiveWalking::GetInstance()->COB_Y_ADJUSTMENT_ABS_MAX_MM = 50;

	RecursiveWalking::GetInstance()->P_GAIN = 64;
	RecursiveWalking::GetInstance()->I_GAIN = 0;
	RecursiveWalking::GetInstance()->D_GAIN = 0;


	////AddModule
	MotionManager::GetInstance()->AddModule((MotionModule*)RecursiveWalking::GetInstance());

	for(unsigned int index = 0; index < MotionStatus::m_CurrentJoints.size(); index++) {
		int id = MotionStatus::m_CurrentJoints[index].m_ID;

		int err;
		MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteDWord(id, PRO54::P_GOAL_ACCELATION_LL, 0, &err);
		MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteDWord(id, PRO54::P_GOAL_VELOCITY_LL, 0, &err);
		printf("id : %d  ", id);
		PrintErrorCode(err);
		usleep(1000);
	}

//	for(unsigned int index = 0; index < MotionStatus::m_CurrentJoints.size(); index++) {
//		int id = MotionStatus::m_CurrentJoints[index].m_ID;
//
//		int err;
//		if(id >= 15 && id <= 26) {
//			MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteWord(id, PRO54::P_VELOCITY_I_GAIN_L, 16, &err);
//
//		}
//
//		if(id == 15 || id == 16 || id == 25 || id == 27) {
//			MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteWord(id, PRO54::P_VELOCITY_I_GAIN_L, 14, &err);
//		}
//		printf("id : %d  ", id);
//		PrintErrorCode(err);
//		usleep(1000);
//	}


	printf("Press the any key. After the fall down on the ground!\n");
	_getch();

	double RollInitAngleRad = 0.0, PitchInitAngleRad = 0.0;
	int InitAngleWindowSize = 50;
	for(int index =0; index < InitAngleWindowSize; index++)	{
		RollInitAngleRad += MotionStatus::EulerAngleX;
		PitchInitAngleRad += MotionStatus::EulerAngleY;
		usleep(4000);
	}
	RecursiveWalking::GetInstance()->SetInitAngleinRad(RollInitAngleRad/((double)InitAngleWindowSize), PitchInitAngleRad/((double)InitAngleWindowSize));

	printf("Setting Init Angle is completed %f %f!\n", RollInitAngleRad/((double)InitAngleWindowSize),  PitchInitAngleRad/((double)InitAngleWindowSize));

	SetTheMotionEnableList();
	MotionManager::GetInstance()->StartTimer();
	RecursiveWalking::GetInstance()->Start();



	while(true)
	{
		__fpurge(stdin);
		int c_temp = _getch();

		if(c_temp == 27) // ESC
		{
			printf("terminated\n");
			break;
		}
		else if( c_temp == 49 ) // 1
		{
			g_stride_length = 10.0;
			g_side_stride_length = 10.0;
			g_stride_degree = 2.0;
			g_stride_radian = g_stride_degree*ANG2RAD;
			printf("stride %f\n", g_stride_length);
		}
		else if( c_temp == 50 ) // 2
		{
			g_stride_length = 50.0;
			g_side_stride_length = 25.0;
			g_stride_degree = 5.0;
			g_stride_radian = g_stride_degree*ANG2RAD;
			printf("stride %f\n", g_stride_length);
		}
		else if( c_temp == 51) // 3
		{
			g_stride_length = 100.0;
			g_side_stride_length = 50.0;
			g_stride_degree = 10.0;
			g_stride_radian =	g_stride_degree*ANG2RAD;
			printf("stride %f\n", g_stride_length);
		}
		else if( c_temp == 'w' || c_temp == 'W')
		{
			UpdateTranslateParamter(g_phase, StepForward);
			printf("g_dsp : %f, g_period : %f, g_y_zmp_convergence : %f\n", g_dsp, g_period, g_y_zmp_convergence);
			UpdateTranslateStepData(g_first_foot_move, g_stride_length, 0.0);
			RecursiveWalking::GetInstance()->AddStepData(g_stp0);
			RecursiveWalking::GetInstance()->AddStepData(g_stp1);
			RecursiveWalking::GetInstance()->AddStepData(g_stp2);
			RecursiveWalking::GetInstance()->AddStepData(g_stp3);
			if(RecursiveWalking::GetInstance()->CalcWalkingPattern() == false)
			{
				RecursiveWalking::GetInstance()->ClearStepData();
				continue;
			}

			RecursiveWalking::GetInstance()->Start();
			while(RecursiveWalking::GetInstance()->IsRunning())
				usleep(8000);

			RecursiveWalking::GetInstance()->ClearStepData();
			RecursiveWalking::GetInstance()->m_ReferenceGtoBodyPosition.z = g_body_height;
			printf("WalkingDone\n");
		}
		else if( c_temp == 'a' || c_temp == 'A')
		{
			UpdateTranslateParamter(g_phase, StepLeft);
			UpdateTranslateStepData(LFootMove, 0.0, g_side_stride_length);
			printf("g_dsp : %f, g_period : %f, g_y_zmp_convergence : %f\n", g_dsp, g_period, g_y_zmp_convergence);
			RecursiveWalking::GetInstance()->AddStepData(g_stp0);
			RecursiveWalking::GetInstance()->AddStepData(g_stp1);
			RecursiveWalking::GetInstance()->AddStepData(g_stp2);
			RecursiveWalking::GetInstance()->AddStepData(g_stp3);
			if(RecursiveWalking::GetInstance()->CalcWalkingPattern() == false)
			{
				RecursiveWalking::GetInstance()->ClearStepData();
				continue;
			}

			RecursiveWalking::GetInstance()->Start();
			while(RecursiveWalking::GetInstance()->IsRunning())
				usleep(8000);

			RecursiveWalking::GetInstance()->ClearStepData();
			RecursiveWalking::GetInstance()->m_ReferenceGtoBodyPosition.z = g_body_height;
			printf("WalkingDone\n");
		}
		else if( c_temp == 'd' || c_temp == 'D')
		{
			UpdateTranslateParamter(g_phase, StepRight);
			UpdateTranslateStepData(RFootMove, 0.0, -g_side_stride_length);
			printf("g_dsp : %f, g_period : %f, g_y_zmp_convergence : %f\n", g_dsp, g_period, g_y_zmp_convergence);
			RecursiveWalking::GetInstance()->AddStepData(g_stp0);
			RecursiveWalking::GetInstance()->AddStepData(g_stp1);
			RecursiveWalking::GetInstance()->AddStepData(g_stp2);
			RecursiveWalking::GetInstance()->AddStepData(g_stp3);
			if(RecursiveWalking::GetInstance()->CalcWalkingPattern() == false)
			{
				RecursiveWalking::GetInstance()->ClearStepData();
				continue;
			}

			RecursiveWalking::GetInstance()->Start();
			while(RecursiveWalking::GetInstance()->IsRunning())
				usleep(8000);

			RecursiveWalking::GetInstance()->ClearStepData();
			RecursiveWalking::GetInstance()->m_ReferenceGtoBodyPosition.z = g_body_height;
			printf("WalkingDone\n");
		}
		else if( c_temp == 'x' || c_temp == 'X')
		{
			UpdateTranslateParamter(g_phase, StepBackrward);
			UpdateTranslateStepData(g_first_foot_move, -g_stride_length, 0.0);
			printf("g_dsp : %f, g_period : %f, g_y_zmp_convergence : %f\n", g_dsp, g_period, g_y_zmp_convergence);
			RecursiveWalking::GetInstance()->AddStepData(g_stp0);
			RecursiveWalking::GetInstance()->AddStepData(g_stp1);
			RecursiveWalking::GetInstance()->AddStepData(g_stp2);
			RecursiveWalking::GetInstance()->AddStepData(g_stp3);
			if(RecursiveWalking::GetInstance()->CalcWalkingPattern() == false)
			{
				RecursiveWalking::GetInstance()->ClearStepData();
				continue;
			}

			RecursiveWalking::GetInstance()->Start();
			while(RecursiveWalking::GetInstance()->IsRunning())
				usleep(8000);

			RecursiveWalking::GetInstance()->ClearStepData();
			RecursiveWalking::GetInstance()->m_ReferenceGtoBodyPosition.z = g_body_height;
			printf("WalkingDone\n");
		}
		//Rotate
		else if( c_temp == 'q' || c_temp == 'Q')
		{
			UpdateRotateParamter(g_phase, StepLeftRotate);
			printf("g_dsp : %f, g_period : %f, g_y_zmp_convergence : %f\n", g_dsp, g_period, g_y_zmp_convergence);
			UpdateRotateStepData(LFootMove, g_stride_degree);
			RecursiveWalking::GetInstance()->AddStepData(g_stp0);
			RecursiveWalking::GetInstance()->AddStepData(g_stp1);
			RecursiveWalking::GetInstance()->AddStepData(g_stp2);
			RecursiveWalking::GetInstance()->AddStepData(g_stp3);
			if(RecursiveWalking::GetInstance()->CalcWalkingPattern() == false)
			{
				RecursiveWalking::GetInstance()->ClearStepData();
				continue;
			}

			RecursiveWalking::GetInstance()->Start();
			while(RecursiveWalking::GetInstance()->IsRunning())
				usleep(8000);

			RecursiveWalking::GetInstance()->ClearStepData();
			RecursiveWalking::GetInstance()->m_ReferenceGtoBodyPosition.yaw = 0.0;
			printf("WalkingDone\n");

		}
		else if( c_temp == 'e' || c_temp == 'E')
		{
			UpdateRotateParamter(g_phase, StepRightRotate);
			printf("g_dsp : %f, g_period : %f, g_y_zmp_convergence : %f\n", g_dsp, g_period, g_y_zmp_convergence);
			UpdateRotateStepData(RFootMove, -g_stride_degree);
			RecursiveWalking::GetInstance()->AddStepData(g_stp0);
			RecursiveWalking::GetInstance()->AddStepData(g_stp1);
			RecursiveWalking::GetInstance()->AddStepData(g_stp2);
			RecursiveWalking::GetInstance()->AddStepData(g_stp3);
			if(RecursiveWalking::GetInstance()->CalcWalkingPattern() == false)
			{
				RecursiveWalking::GetInstance()->ClearStepData();
				continue;
			}

			RecursiveWalking::GetInstance()->Start();
			while(RecursiveWalking::GetInstance()->IsRunning())
				usleep(8000);

			RecursiveWalking::GetInstance()->ClearStepData();
			RecursiveWalking::GetInstance()->m_ReferenceGtoBodyPosition.yaw = 0.0;
			printf("WalkingDone\n");
		}
		else if( c_temp == 'c' || c_temp == 'C')
		{
			UpdateRotateParamter(g_phase, StepRightRotate);
			printf("g_dsp : %f, g_period : %f, g_y_zmp_convergence : %f\n", g_dsp, g_period, g_y_zmp_convergence);
			UpdateRotateStepData(LFootMove, -g_stride_degree);
			RecursiveWalking::GetInstance()->AddStepData(g_stp0);
			RecursiveWalking::GetInstance()->AddStepData(g_stp1);
			RecursiveWalking::GetInstance()->AddStepData(g_stp2);
			RecursiveWalking::GetInstance()->AddStepData(g_stp3);
			if(RecursiveWalking::GetInstance()->CalcWalkingPattern() == false)
			{
				RecursiveWalking::GetInstance()->ClearStepData();
				continue;
			}

			RecursiveWalking::GetInstance()->Start();
			while(RecursiveWalking::GetInstance()->IsRunning())
				usleep(8000);

			RecursiveWalking::GetInstance()->ClearStepData();
			RecursiveWalking::GetInstance()->m_ReferenceGtoBodyPosition.yaw = 0.0;
			printf("WalkingDone\n");
		}
		else if( c_temp == 'z' || c_temp == 'Z')
		{
			UpdateRotateParamter(g_phase, StepLeftRotate);
			printf("g_dsp : %f, g_period : %f, g_y_zmp_convergence : %f\n", g_dsp, g_period, g_y_zmp_convergence);
			UpdateRotateStepData(RFootMove, g_stride_degree);
			RecursiveWalking::GetInstance()->AddStepData(g_stp0);
			RecursiveWalking::GetInstance()->AddStepData(g_stp1);
			RecursiveWalking::GetInstance()->AddStepData(g_stp2);
			RecursiveWalking::GetInstance()->AddStepData(g_stp3);
			if(RecursiveWalking::GetInstance()->CalcWalkingPattern() == false)
			{
				RecursiveWalking::GetInstance()->ClearStepData();
				continue;
			}

			RecursiveWalking::GetInstance()->Start();
			while(RecursiveWalking::GetInstance()->IsRunning())
				usleep(8000);

			RecursiveWalking::GetInstance()->ClearStepData();
			RecursiveWalking::GetInstance()->m_ReferenceGtoBodyPosition.yaw = 0.0;
			printf("WalkingDone\n");
		}
		else if(c_temp == 'u' || c_temp == 'U')
		{




		}
		else if(c_temp == 'r' || c_temp == 'R')
		{
			if(g_first_foot_move == RFootMove)
			{
				printf("Left Foot First\n");
				g_first_foot_move = LFootMove;
			}
			else if(g_first_foot_move == LFootMove)
			{
				g_first_foot_move = RFootMove;
				printf("Right Foot First\n");
			}
		}
		else if(c_temp == 55) //7
		{
			 g_dsp += 0.01;
			 printf("g_dsp : %f\n", g_dsp);
		}
		else if(c_temp == 54) //6
		{
			 g_dsp -= 0.01;
			 printf("g_dsp : %f\n", g_dsp);
		}
		else if(c_temp == 57) //9
		{
			 g_period += 40;
			 printf("g_period : %f\n", g_period);
		}
		else if(c_temp == 56) //8
		{
			g_period -= 40;
			printf("g_period : %f\n", g_period);
		}
	}
	return 0;
}

void UpdateTranslateParamter(int phase, int kind_of_step)
{
	int l_phase = phase;
	if(kind_of_step == StepForward) {
		if(l_phase == first_flat) {
			g_period = 850;
			g_dsp = 0.3;
			g_y_zmp_convergence = 15.0+3.0;
		}
		else if(l_phase == on_the_ramp) {
			g_period = 850;
			g_dsp = 0.3;
			g_y_zmp_convergence = 15.0+3.0;
		}
		else if(l_phase == on_the_ramp1) {
			g_period = 850;
			g_dsp = 0.3;
			g_y_zmp_convergence = 15.0+3.0;
		}
		else {
			g_period = 850;
			g_dsp = 0.3;
			g_y_zmp_convergence = 15.0+3.0;
		}
	}
	else if(kind_of_step == StepBackrward)
	{
		if(l_phase == first_flat) {
			g_period = 850;
			g_dsp = 0.3;
			g_y_zmp_convergence = 15.0+3.0;
		}
		else if(l_phase == on_the_ramp) {
			g_period = 850;
			g_dsp = 0.3;
			g_y_zmp_convergence = 15.0+3.0;
		}
		else if(l_phase == on_the_ramp1) {
			g_period = 850;
			g_dsp = 0.3;
			g_y_zmp_convergence = 15.0+3.0;
		}
		else {
			g_period = 850;
			g_dsp = 0.3;
			g_y_zmp_convergence = 15.0+3.0;
		}
	}
	else if(kind_of_step == StepRight)
	{
		if(l_phase == first_flat) {
			g_period = 850;
			g_dsp = 0.3;
			g_y_zmp_convergence = 25.0+3.0;
		}
		else if(l_phase == on_the_ramp) {
			g_period = 850;
			g_dsp = 0.3;
			g_y_zmp_convergence = 25.0+3.0;
		}
		else if(l_phase == on_the_ramp1) {
			g_period = 850;
			g_dsp = 0.3;
			g_y_zmp_convergence = 25.0+3.0;
		}
		else {
			g_period = 850;
			g_dsp = 0.3;
			g_y_zmp_convergence = 25.0+3.0;
		}
	}
	else if(kind_of_step == StepLeft)
	{
		if(l_phase == first_flat) {
			g_period = 850;
			g_dsp = 0.3;
			g_y_zmp_convergence = 25.0+3.0;
		}
		else if(l_phase == on_the_ramp) {
			g_period = 850;
			g_dsp = 0.3;
			g_y_zmp_convergence = 25.0+3.0;
		}
		else if(l_phase == on_the_ramp1) {
			g_period = 850;
			g_dsp = 0.3;
			g_y_zmp_convergence = 25.0+3.0;
		}
		else {
			g_period = 850;
			g_dsp = 0.3;
			g_y_zmp_convergence = 25.0+3.0;
		}
	}
	RecursiveWalking::GetInstance()->SetRefZMPDecisionParameter(0.0, 0.0, g_y_zmp_convergence);
}

void UpdateRotateParamter(int phase, int kind_of_step)
{
	int l_phase = phase;
	if(kind_of_step == StepRightRotate) {
		if(l_phase == first_flat) {
			g_period = 850;
			g_dsp = 0.3;
			g_y_zmp_convergence = 13.0+3.0;
		}
		else if(l_phase == on_the_ramp) {
			g_period = 850;
			g_dsp = 0.3;
			g_y_zmp_convergence = 13.0+3.0;
		}
		else if(l_phase == on_the_ramp1) {
			g_period = 850;
			g_dsp = 0.3;
			g_y_zmp_convergence = 13.0+3.0;
		}
		else{
			g_period = 850;
			g_dsp = 0.3;
			g_y_zmp_convergence = 13.0+3.0;
		}
	}
	else if(kind_of_step == StepLeftRotate)
	{
		if(l_phase == first_flat) {
			g_period = 850;
			g_dsp = 0.3;
			g_y_zmp_convergence = 13.0+3.0;
		}
		else if(l_phase == on_the_ramp) {
			g_period = 850;
			g_dsp = 0.3;
			g_y_zmp_convergence = 13.0+3.0;
		}
		else if(l_phase == on_the_ramp1) {
			g_period = 850;
			g_dsp = 0.3;
			g_y_zmp_convergence = 13.0+3.0;
		}
		else
		{
			g_period = 850;
			g_dsp = 0.3;
			g_y_zmp_convergence = 13.0+3.0;
		}
	}
	RecursiveWalking::GetInstance()->SetRefZMPDecisionParameter(0.0, 0.0, g_y_zmp_convergence);
}
