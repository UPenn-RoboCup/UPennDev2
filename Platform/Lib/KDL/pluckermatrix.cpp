#include "pluckermatrix.hpp"
#include "spatialvector.hpp"
#include <cassert>

// Mike Hopkins 2/15/2013
// based on source code from http://royfeatherstone.org/spatial/v2/
// Spatial Vector and Rigid-Body Dynamics Software by Roy Featherstone

using namespace Eigen;

namespace
{
  inline Matrix3d ArraytoMatrix3d(const double data[9])
  {
    Matrix3d M;
    M(0, 0) = data[0];
    M(0, 1) = data[1];
    M(0, 2) = data[2];
    M(1, 0) = data[3];
    M(1, 1) = data[4];
    M(1, 2) = data[5];
    M(2, 0) = data[6];
    M(2, 1) = data[7];
    M(2, 2) = data[8];
    return M;
  }

  inline void Matrix3dtoArray(const Matrix3d &M, double data[9])
  {
    data[0] = M(0, 0);
    data[1] = M(0, 1);
    data[2] = M(0, 2);
    data[3] = M(1, 0);
    data[4] = M(1, 1);
    data[5] = M(1, 2);
    data[6] = M(2, 0);
    data[7] = M(2, 1);
    data[8] = M(2, 2);
  }

  inline Vector3d ArraytoVector3d(const double data[3])
  {
    Vector3d v;
    v(0) = data[0];
    v(1) = data[1];
    v(2) = data[2];
    return v;
  }

  inline void Vector3dtoArray(const Vector3d &v, double data[3])
  {
    data[0] = v(0);
    data[1] = v(1);
    data[2] = v(2);
  }

  inline Matrix3d skew(const Vector3d &v)
  {
      Matrix3d S;
      S(0, 0) = 0;
      S(0, 1) = -v[2];
      S(0, 2) = v[1];
      S(1, 0) = v[2];
      S(1, 1) = 0;
      S(1, 2) = -v[0];
      S(2, 0) = -v[1];
      S(2, 1) = v[0];
      S(2, 2) = 0;
      return S;
  }

  inline Vector3d unskew(const Matrix3d &S)
  {
      Vector3d v;
      v(0) = 0.5*(S(2, 1) - S(1, 2));
      v(1) = 0.5*(S(0, 2) - S(2, 0));
      v(2) = 0.5*(S(1, 0) - S(0, 1));
      return v;
  }
}

namespace KDL
{
    PluckerMatrix::PluckerMatrix()
    {
      X = Matrix6d::Identity();
    }
 
    PluckerMatrix::PluckerMatrix(const Matrix6d &X_)
    {
      assert(X_.rows() == 6 && X_.cols() == 6);
      X = X_;
    }

    PluckerMatrix::PluckerMatrix(const Frame &F)
    {
	X = Matrix6d::Zero();
        Matrix3d E = ArraytoMatrix3d(F.M.data);
	Vector3d mEr = ArraytoVector3d(F.p.data);
	X.topLeftCorner(3, 3) = E;
	X.bottomLeftCorner(3, 3) = skew(mEr)*E;
	X.bottomRightCorner(3, 3) = E;
    }

    PluckerMatrix::PluckerMatrix(RigidBodyInertia I)
    {
	X = Matrix6d::Zero();
        Vector cog = I.getCOG();
        I = I.RefPoint(cog);
	double m = I.getMass();
	Vector3d c = ArraytoVector3d(cog.data);
	Matrix3d C = skew(c);
        Matrix3d Ic = ArraytoMatrix3d(I.getRotationalInertia().data);
        X.topLeftCorner(3, 3) = Ic + m*C*C.transpose();
        X.topRightCorner(3, 3) = m*C;
        X.bottomLeftCorner(3, 3) = m*C.transpose();
        X.bottomRightCorner(3, 3) = m*Matrix3d::Identity();
    }
 
    Frame PluckerMatrix::toFrame() const
    {
	Frame F;
	Matrix3d E = X.topLeftCorner(3, 3);
	Matrix3d mErx = X.bottomLeftCorner(3, 3);
	Vector3d mEr = unskew(mErx*E.transpose());
        Vector3dtoArray(mEr, F.p.data);
        Matrix3dtoArray(E, F.M.data);
	return F;
    }

    RigidBodyInertia PluckerMatrix::toRigidBodyInertia() const
    {
        double m = X(5, 5);
        Matrix3d mC = X.topRightCorner(3, 3); 
        Vector3d c = unskew(mC)/m;
        Matrix3d Ic = X.topLeftCorner(3, 3) - mC*mC.transpose()/m;
        Vector cog;
        RotationalInertia I;
        Vector3dtoArray(c, cog.data);
        Matrix3dtoArray(Ic, I.data);
	return RigidBodyInertia(m, cog, I);
    }

    PluckerMatrix PluckerMatrix::Inverse() const
    {
      return PluckerMatrix(X.inverse());
    }

    PluckerMatrix PluckerMatrix::Transpose() const
    {
      return PluckerMatrix(X.transpose());
    }

    SpatialVector PluckerMatrix::operator * (const SpatialVector &v) const
    {
      return SpatialVector(X*v.v);
    }

    PluckerMatrix PluckerMatrix::operator * (const PluckerMatrix &Y) const
    {
      return PluckerMatrix(X*Y.X);
    }
}
