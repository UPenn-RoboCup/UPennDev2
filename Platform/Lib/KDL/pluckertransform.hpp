#ifndef KDL_PLUCKER_TRANSFORM_HPP
#define KDL_PLUCKER_TRANSFORM_HPP

#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <Eigen/Core>
#include <Eigen/LU>

using namespace Eigen;

namespace KDL
{
    typedef Matrix<double, 6, 6> Matrix6d;

    class SpatialVector;

    class PluckerTransform {
    public:
        Matrix6d X;

        PluckerTransform();
        PluckerTransform(const Matrix6d &X_);
        PluckerTransform(const Frame &F);
        PluckerTransform(RigidBodyInertia I);

        Frame toFrame() const;
        RigidBodyInertia toRigidBodyInertia() const;
        PluckerTransform Inverse() const;
        PluckerTransform Transpose() const;

        SpatialVector operator * (const SpatialVector &v) const;
        PluckerTransform operator * (const PluckerTransform &Y) const;
    };
}

#endif
