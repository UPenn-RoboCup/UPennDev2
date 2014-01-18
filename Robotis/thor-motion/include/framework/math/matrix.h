/*
 * matrix.h
 *
 *  Created on: 2013. 2. 4.
 *      Author: hjsong
 */

#ifndef MATRIX_H_
#define MATRIX_H_

#include "math/vector.h"
#include "math/point.h"

namespace Thor
{

class Matrix3D
{
public:
	enum
	{
		m00 = 0,
		m01,
		m02,
		m03,
		m10,
		m11,
		m12,
		m13,
		m20,
		m21,
		m22,
		m23,
		m30,
		m31,
		m32,
		m33,
		MAXNUM_ELEMENT
	};

private:

protected:

public:
	double m[MAXNUM_ELEMENT]; // Element

	Matrix3D();
	Matrix3D(const Matrix3D &mat);
	~Matrix3D();

	void Identity();
	bool Inverse();
	void Scale(Vector3D scale);
	void Rotate(double angle, Vector3D axis);
	void Translate(Vector3D offset);
	Point3D Transform(Point3D point);
	Vector3D Transform(Vector3D vector);
	void SetTransform(Point3D point, Vector3D angle);

	Matrix3D & operator = (const Matrix3D &mat);
	Matrix3D & operator *= (const Matrix3D &mat);
	Matrix3D operator * (const Matrix3D &mat);
};


}

#endif /* MATRIX_H_ */
