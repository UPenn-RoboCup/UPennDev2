/*
 * Algebra.h
 *
 *  Created on: 2013. 12. 3.
 *      Author: hjsong
 */

#ifndef LINEAR_ALGEBRA_H_
#define LINEAR_ALGEBRA_H_

#include <math.h>
#include <stdio.h>
#include "step_data_define.h"
#include "Eigen/Eigen/Dense"

namespace Thor
{
	typedef Eigen::MatrixXd matd;
	typedef Eigen::MatrixXi mati;

	typedef Eigen::VectorXd vecd;
	typedef Eigen::VectorXi veci;

	matd GetOrientationMatrix(double Roll, double Pitch, double Yaw);
	matd GetTranslationMatrix(double x, double y, double z);
	matd GetTransformMatrix(double x, double y, double z, double Roll, double Pitch, double Yaw);
	matd GetTransformMatrixInverse(matd T);
	Pose3D GetPose3DfromTransformMatrix(matd matTransform);
	vecd Cross(vecd v1, vecd v2);
	double Dot(vecd v1, vecd v2);


	matd CalcDH(double link_length_mm, double link_twist_rad, double joint_offset_mm, double joint_angle_rad);


	double min( double a, double b);
	double max( double a, double b);
	double sign(double a);
	double abs(double a);

}

#endif /* ALGEBRA_H_ */
