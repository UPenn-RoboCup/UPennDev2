#include "pluckertransform.hpp"
#include "spatialvector.hpp"
#include <cassert>

// Mike Hopkins 2/15/2013
// based on source code from http://royfeatherstone.org/spatial/v2/
// Spatial Vector and Rigid-Body Dynamics Software by Roy Featherstone

using namespace Eigen;

namespace
{
  Matrix3d skew(Vector3d v)
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

  Vector3d unskew(Matrix3d S)
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
    PluckerTransform::PluckerTransform()
    {
      X = Matrix6d::Identity();
    }
 
    PluckerTransform::PluckerTransform(const Matrix6d &X_)
    {
      assert(X_.rows() == 6 && X_.cols() == 6);
      X = X_;
    }

    PluckerTransform::PluckerTransform(const Frame &F)
    {
	X = Matrix6d::Zero();
	Matrix3d E = Map<const Matrix3d, ColMajor>(F.M.data);
	Vector3d mEr = Map<const Vector3d>(F.p.data);
	X.topLeftCorner(3, 3) = E;
	X.bottomLeftCorner(3, 3) = skew(mEr)*E;
	X.bottomRightCorner(3, 3) = E;
    }

    PluckerTransform::PluckerTransform(RigidBodyInertia I)
    {
	X = Matrix6d::Zero();
        Vector cog = I.getCOG();
        I = I.RefPoint(cog);
	double m = I.getMass();
	Vector3d c = Map<const Vector3d>(cog.data);
	Matrix3d C = skew(c);
	Matrix3d Ic = Map<const Matrix3d, ColMajor>(I.getRotationalInertia().data);
        X.topLeftCorner(3, 3) = Ic + m*C*C.transpose();
        X.topRightCorner(3, 3) = m*C;
        X.bottomLeftCorner(3, 3) = m*C.transpose();
        X.bottomRightCorner(3, 3) = m*Matrix3d::Identity();
    }
 
    Frame PluckerTransform::toFrame() const
    {
	Frame F;
	Matrix3d E = X.topLeftCorner(3, 3);
	Matrix3d mErx = X.bottomLeftCorner(3, 3);
	Vector3d mEr = unskew(mErx*E.transpose());
        Map<Vector3d>(F.p.data) = mEr;
        Map<Matrix3d, ColMajor>(F.M.data) = E;
	return F;
    }

    RigidBodyInertia PluckerTransform::toRigidBodyInertia() const
    {
        double m = X(5, 5);
        Matrix3d mC = X.topRightCorner(3, 3); 
        Vector3d c = unskew(mC)/m;
        Matrix3d Ic = X.topLeftCorner(3, 3) - mC*mC.transpose()/m;
        Vector cog;
        RotationalInertia I;
        Map<Vector3d>(cog.data) = c;
        Map<Matrix3d, ColMajor>(I.data) = Ic;
	return RigidBodyInertia(m, cog, I);
    }

    PluckerTransform PluckerTransform::Inverse() const
    {
      return PluckerTransform(X.inverse());
    }

    PluckerTransform PluckerTransform::Transpose() const
    {
      return PluckerTransform(X.transpose());
    }

    SpatialVector PluckerTransform::operator * (const SpatialVector &v) const
    {
      return SpatialVector(X*v.v);
    }

    PluckerTransform PluckerTransform::operator * (const PluckerTransform &Y) const
    {
      return PluckerTransform(X*Y.X);
    }
}
