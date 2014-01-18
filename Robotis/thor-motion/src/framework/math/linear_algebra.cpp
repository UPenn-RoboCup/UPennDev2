/*
 * LinearAlgebra.cpp
 *
 *  Created on: 2013. 12. 3.
 *      Author: hjsong
 */
#include "framework/math/linear_algebra.h"

using namespace Thor;

double Thor::min( double a, double b)
{
	if (a < b)
		return a;
	else
		return b;
}

double Thor::max( double a, double b)
{
	if (a > b)
		return a;
	else
		return b;
}

double Thor::sign(double a)
{
	if(a < 0.0)
		return -1.0;
	else
		return 1.0;
}

double Thor::abs(double a)
{
	if(a < 0.0)
		return -a;
	else
		return a;
}

matd Thor::GetOrientationMatrix(double Roll, double Pitch, double Yaw)
{
	double sr = sin(Roll), cr = cos(Roll);
	double sp = sin(Pitch), cp = cos(Pitch);
	double sy = sin(Yaw), cy = cos(Yaw);

	matd matRoll(4,4);
	matd matPitch(4,4);
	matd matYaw(4,4);

	matRoll << 1,   0,   0,  0,
			0,  cr, -sr,  0,
			0,  sr,  cr,  0,
			0,   0,   0,  1;

	matPitch << cp,  0,  sp,  0,
				 0,  1,   0,  0,
			   -sp,  0,  cp,  0,
				 0,  0,   0,  1;

	matYaw << cy, -sy,  0,  0,
			  sy,  cy,  0,  0,
			   0,   0,  1,  0,
			   0,   0,  0,  1;


	return matYaw*matPitch*matRoll;

}

matd Thor::GetTranslationMatrix(double x, double y, double z)
{
	matd matTranslation;
	matTranslation.setIdentity(4,4);

	matTranslation(0, 3) = x;
	matTranslation(1, 3) = y;
	matTranslation(2, 3) = z;

	return matTranslation;
}

matd Thor::GetTransformMatrix(double x, double y, double z, double Roll, double Pitch, double Yaw)
{
	matd matTransform = Thor::GetOrientationMatrix(Roll, Pitch, Yaw);
	matTransform(0, 3) = x;
	matTransform(1, 3) = y;
	matTransform(2, 3) = z;

	return matTransform;
}

matd Thor::GetTransformMatrixInverse(matd T)
{
	vecd vecBOA(3); //If T is Transform Matrix A from B, the BOA is translation component coordi. B to coordi. A
	vecd vec_x(3); vecd vec_y(3); vecd vec_z(3);
	matd invT(4,4);

	vecBOA(0) = -T(0,3); vecBOA(1) = -T(1,3); vecBOA(2) = -T(2,3);
	vec_x(0) = T(0,0); vec_x(1) = T(1,0); vec_x(2) = T(2,0);
	vec_y(0) = T(0,1); vec_y(1) = T(1,1); vec_y(2) = T(2,1);
	vec_z(0) = T(0,2); vec_z(1) = T(1,2); vec_z(2) = T(2,2);


	// inv = [   x'   | -AtoB¡¤x ]
	//       [   y'   | -AtoB¡¤y ]
	//       [   z'   | -AtoB¡¤z ]
	//       [  0 0 0  |       1 ]

	invT << vec_x(0), vec_x(1), vec_x(2), Dot(vecBOA, vec_x),
			vec_y(0), vec_y(1), vec_y(2), Dot(vecBOA, vec_y),
			vec_z(0), vec_z(1), vec_z(2), Dot(vecBOA, vec_z),
			      0,        0,        0,                   1;

	return invT;
}

Pose3D Thor::GetPose3DfromTransformMatrix(matd matTransform)
{
	Pose3D _tempPose3D;

	_tempPose3D.x     = matTransform(0, 3);
	_tempPose3D.y     = matTransform(1, 3);
	_tempPose3D.z     = matTransform(2, 3);
	_tempPose3D.roll  = atan2(matTransform(1,2), matTransform(2,2));
	_tempPose3D.pitch = atan2(-matTransform(2,0), sqrt(matTransform(2,1)*matTransform(2,1) + matTransform(2,2)*matTransform(2,2)));
	_tempPose3D.yaw   = atan2(matTransform(1,0), matTransform(0,0));

	return _tempPose3D;
}

vecd Thor::Cross(vecd v1, vecd v2)
{
	vecd result(3);
	result(0) = v1(1) * v2(2) - v1(2) * v2(1);
	result(1) = v1(2) * v2(0) - v1(0) * v2(2);
	result(2) = v1(0) * v2(1) - v1(1) * v2(0);

	return result;
}

double Thor::Dot(vecd v1, vecd v2)
{

	double result = 0;
	if(v1.size() != v2.size() )
	{
		//fprintf(stderr, "The vector size Must be the same.\n");
		return result;
	}
	else
	{
		for(int vec_idx = 0; vec_idx < v1.size(); vec_idx++) {
			result += v1(vec_idx)*v2(vec_idx);
		}

		return result;
	}
}

matd Thor::CalcDH(double link_length_mm, double link_twist_rad, double joint_offset_mm, double joint_angle_rad)
{
	matd T(4,4);
	double l = link_length_mm;
	double a = link_twist_rad;
	double d = joint_offset_mm;
	double th = joint_angle_rad;

	T << cos(th),   -cos(a)*sin(th),    sin(a)*sin(th),   l*cos(th),
	     sin(th),    cos(a)*cos(th),   -sin(a)*cos(th),   l*sin(th),
	         0,             sin(a),            cos(a),           d,
	         0,                 0,                 0,            1;

	return T;
}

