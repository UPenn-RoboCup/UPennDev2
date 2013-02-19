#include "spatialvector.hpp"
#include "pluckermatrix.hpp"
#include <cassert>

// Mike Hopkins 2/15/2013

using namespace Eigen;

namespace KDL
{
    SpatialVector::SpatialVector()
    {
      v = VectorXd::Zero(6);
    }
 
    SpatialVector::SpatialVector(const VectorXd &v_)
    {
      assert(v_.rows() == 6);
      v = v_;
    }

    SpatialVector::SpatialVector(const Twist &t)
    {
	v = VectorXd::Zero(6);
        v(0) = t.rot(0);
        v(1) = t.rot(1); 
        v(2) = t.rot(2);
        v(3) = t.vel(0); 
        v(4) = t.vel(1);
        v(5) = t.vel(2);
    }

    SpatialVector::SpatialVector(const Wrench &w) 
    {
	v = VectorXd::Zero(6);
        v(0) = w.torque(0);
        v(1) = w.torque(1); 
        v(2) = w.torque(2);
        v(3) = w.force(0); 
        v(4) = w.force(1);
        v(5) = w.force(2);
    }
 
    Twist SpatialVector::toTwist() const
    {
        Twist t;
        t.rot(0) = v(0);
        t.rot(1) = v(1);
        t.rot(2) = v(2);
        t.vel(0) = v(3);
        t.vel(1) = v(4);
        t.vel(2) = v(5);
	return t;
    }

    Wrench SpatialVector::toWrench() const
    {
	Wrench w;
        w.torque(0) = v(0);
        w.torque(1) = v(1);
        w.torque(2) = v(2);
        w.force(0) = v(3);
        w.force(1) = v(4);
        w.force(2) = v(5);
	return w;
    }

    SpatialVector SpatialVector::operator * (const PluckerMatrix &X) const
    {
      return SpatialVector((v.transpose()*X.X).transpose());
    }
}
