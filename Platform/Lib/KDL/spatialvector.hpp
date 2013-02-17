#ifndef KDL_SPATIAL_VECTOR_HPP
#define KDL_SPATIAL_VECTOR_HPP

#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <Eigen/Core>
#include <Eigen/LU>

using namespace Eigen;

namespace KDL
{
    class PluckerMatrix;

    class SpatialVector {
    public:
        VectorXd v;

        SpatialVector();
        SpatialVector(const VectorXd &v_);
        SpatialVector(const Twist &t);
        SpatialVector(const Wrench &w);

        Wrench toWrench() const;
        Twist toTwist() const;
        SpatialVector operator * (const PluckerMatrix &X) const;
    };
}

#endif
