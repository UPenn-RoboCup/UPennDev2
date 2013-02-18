#ifndef KDL_PLUCKER_MATRIX_HPP
#define KDL_PLUCKER_MATRIX_HPP

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

    class PluckerMatrix {
    public:
        Matrix6d X;

        PluckerMatrix();
        PluckerMatrix(const Matrix6d &X_);
        PluckerMatrix(const Frame &F);
        PluckerMatrix(RigidBodyInertia I);

        Frame toFrame() const;
        RigidBodyInertia toRigidBodyInertia() const;
        PluckerMatrix Inverse() const;
        PluckerMatrix Transpose() const;

        SpatialVector operator * (const SpatialVector &v) const;
        PluckerMatrix operator * (const PluckerMatrix &Y) const;
    };
}

#endif
