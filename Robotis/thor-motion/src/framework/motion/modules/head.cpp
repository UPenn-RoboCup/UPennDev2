/*
 * head.cpp
 *
 *  Created on: 2013. 3. 7.
 *      Author: hjsong
 */

#include "motion/modules/head.h"
#include "motion/kinematics.h"


using namespace Thor;

Head* Head::m_UniqueInstance = new Head();

#define ID_PANNING_HEAD     29
#define ID_TILTING_HEAD     30

Head::Head()
{
	uID = "Head";
	m_Pan_p_gain = 0.08;
	m_Pan_d_gain = 0.22;

    m_Tilt_p_gain = 0.1;
	m_Tilt_d_gain = 0.22;

	m_LeftLimit = 50.0;
	m_RightLimit = -50.0;
	m_TopLimit = Kinematics::EYE_TILT_OFFSET_ANGLE + 45.0;
	m_BottomLimit = Kinematics::EYE_TILT_OFFSET_ANGLE - 60.0;

	m_Pan_Home = 0.0;
	m_Tilt_Home = Kinematics::EYE_TILT_OFFSET_ANGLE;

	m_Pan_JointIndex = -1;
	m_Tilt_JointIndex = -1;
	//m_Joint.SetEnableHeadOnly(true);
}

Head::~Head()
{  }

void Head::CheckLimit()
{
	if(m_PanAngle > m_LeftLimit)
		m_PanAngle = m_LeftLimit;
	else if(m_PanAngle < m_RightLimit)
		m_PanAngle = m_RightLimit;

	if(m_TiltAngle > m_TopLimit)
		m_TiltAngle = m_TopLimit;
	else if(m_TiltAngle < m_BottomLimit)
		m_TiltAngle = m_BottomLimit;
}

void Head::Initialize()
{
	if(MotionStatus::m_CurrentJoints.size() != 0)
		m_RobotInfo = MotionStatus::m_CurrentJoints;
	else
	{
		fprintf(stderr,"MotionStatus is not initialized");
		return;
	}

	for(unsigned int jointIndex = 0 ; jointIndex < MotionStatus::m_CurrentJoints.size() ; jointIndex++)
	{
		if(MotionStatus::m_CurrentJoints[jointIndex].m_ID == ID_PANNING_HEAD )
		{
			m_PanAngle = MotionStatus::m_CurrentJoints[jointIndex].m_DXLInfo->Value2Angle(MotionStatus::m_CurrentJoints[jointIndex].m_Value);
			m_Pan_JointIndex = jointIndex;
		}
		else if(MotionStatus::m_CurrentJoints[jointIndex].m_ID == ID_TILTING_HEAD)
		{
			m_TiltAngle = MotionStatus::m_CurrentJoints[jointIndex].m_DXLInfo->Value2Angle(MotionStatus::m_CurrentJoints[jointIndex].m_Value);
			m_Tilt_JointIndex = jointIndex;
		}
		else if( (m_Pan_JointIndex != -1) && (m_Tilt_JointIndex != -1) )
			break;
	}

	CheckLimit();

	InitTracking();
	MoveToHome();
}

void Head::LoadINISettings(minIni* ini)
{
    LoadINISettings(ini, HEAD_SECTION);
}

void Head::LoadINISettings(minIni* ini, const std::string &section)
{
    double value = INVALID_VALUE;

    if((value = ini->getd(section, "pan_p_gain", INVALID_VALUE)) != INVALID_VALUE)  m_Pan_p_gain = value;
    if((value = ini->getd(section, "pan_d_gain", INVALID_VALUE)) != INVALID_VALUE)  m_Pan_d_gain = value;
    if((value = ini->getd(section, "tilt_p_gain", INVALID_VALUE)) != INVALID_VALUE) m_Tilt_p_gain = value;
    if((value = ini->getd(section, "tilt_d_gain", INVALID_VALUE)) != INVALID_VALUE) m_Tilt_d_gain = value;
    if((value = ini->getd(section, "left_limit", INVALID_VALUE)) != INVALID_VALUE)  m_LeftLimit = value;
    if((value = ini->getd(section, "right_limit", INVALID_VALUE)) != INVALID_VALUE) m_RightLimit = value;
    if((value = ini->getd(section, "top_limit", INVALID_VALUE)) != INVALID_VALUE)   m_TopLimit = value;
    if((value = ini->getd(section, "bottom_limit", INVALID_VALUE)) != INVALID_VALUE)m_BottomLimit = value;
    if((value = ini->getd(section, "pan_home", INVALID_VALUE)) != INVALID_VALUE)    m_Pan_Home = value;
    if((value = ini->getd(section, "tilt_home", INVALID_VALUE)) != INVALID_VALUE)   m_Tilt_Home = value;
}

void Head::SaveINISettings(minIni* ini)
{
    SaveINISettings(ini, HEAD_SECTION);
}

void Head::SaveINISettings(minIni* ini, const std::string &section)
{
    ini->put(section,   "pan_p_gain",   m_Pan_p_gain);
    ini->put(section,   "pan_d_gain",   m_Pan_d_gain);
    ini->put(section,   "tilt_p_gain",  m_Tilt_p_gain);
    ini->put(section,   "tilt_d_gain",  m_Tilt_d_gain);
    ini->put(section,   "left_limit",   m_LeftLimit);
    ini->put(section,   "right_limit",  m_RightLimit);
    ini->put(section,   "top_limit",    m_TopLimit);
    ini->put(section,   "bottom_limit", m_BottomLimit);
    ini->put(section,   "pan_home",     m_Pan_Home);
    ini->put(section,   "tilt_home",    m_Tilt_Home);
}

void Head::SetPanDXLPgain(int pgain)
{
	if(m_Pan_JointIndex == -1)
		return;
	else
		m_RobotInfo[m_Pan_JointIndex].m_Pgain = pgain;
}

void Head::SetTiltDXLPgain(int pgain)
{
	if(m_Tilt_JointIndex == -1)
		return;
	else
		m_RobotInfo[m_Tilt_JointIndex].m_Pgain = pgain;
}

void Head::MoveToHome()
{
	MoveByAngle(m_Pan_Home, m_Tilt_Home);
}

void Head::MoveByAngle(double pan, double tilt)
{
	m_PanAngle = pan;
	m_TiltAngle = tilt;

	CheckLimit();
}

void Head::MoveByAngleOffset(double pan, double tilt)
{
	MoveByAngle(m_PanAngle + pan, m_TiltAngle + tilt);
}

void Head::InitTracking()
{
	m_Pan_err = 0;
	m_Pan_err_diff = 0;
	m_Tilt_err = 0;
	m_Tilt_err_diff = 0;
}

void Head::MoveTracking(Point2D err)
{
	m_Pan_err_diff = err.X - m_Pan_err;
	m_Pan_err = err.X;

	m_Tilt_err_diff = err.Y - m_Tilt_err;
	m_Tilt_err = err.Y;

	MoveTracking();
}

void Head::MoveTracking()
{
	double pOffset, dOffset;

	pOffset = m_Pan_err * m_Pan_p_gain;
	pOffset *= pOffset;
	if(m_Pan_err < 0)
		pOffset = -pOffset;
	dOffset = m_Pan_err_diff * m_Pan_d_gain;
	dOffset *= dOffset;
	if(m_Pan_err_diff < 0)
		dOffset = -dOffset;
	m_PanAngle += (pOffset + dOffset);

	pOffset = m_Tilt_err * m_Tilt_p_gain;
	pOffset *= pOffset;
	if(m_Tilt_err > 0)
		pOffset = -pOffset;
	dOffset = m_Tilt_err_diff * m_Tilt_d_gain;
	dOffset *= dOffset;
	if(m_Tilt_err_diff > 0)
		dOffset = -dOffset;
	m_TiltAngle += (pOffset + dOffset);

	CheckLimit();
}

void Head::Process()
{
	m_RobotInfo[m_Pan_JointIndex].m_Value = m_RobotInfo[m_Pan_JointIndex].m_DXLInfo->Angle2Value(m_PanAngle);

	m_RobotInfo[m_Tilt_JointIndex].m_Value = m_RobotInfo[m_Tilt_JointIndex].m_DXLInfo->Angle2Value(m_TiltAngle);
	m_RobotInfo[m_Tilt_JointIndex].m_Pgain = 8;
}
