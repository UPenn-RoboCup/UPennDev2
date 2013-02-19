// Copyright  (C)  2009  Ruben Smits <ruben dot smits at mech dot kuleuven dot be>

// Version: 1.0
// Author: Ruben Smits <ruben dot smits at mech dot kuleuven dot be>
// Maintainer: Ruben Smits <ruben dot smits at mech dot kuleuven dot be>
// URL: http://www.orocos.org/kdl

// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.

// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.

// You should have received a copy of the GNU Lesser General Public
// License along with this library; if not, write to the Free Software
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

#ifndef KDL_TREE_ID_FB_SOLVER_RECURSIVE_NEWTON_EULER_HPP
#define KDL_TREE_ID_FB_SOLVER_RECURSIVE_NEWTON_EULER_HPP

#include "treeidfbsolver.hpp"

namespace KDL{
    /**
     * \brief Recursive newton euler floating base inverse dynamics solver
     *
     * The algorithm implementation is based on the book "Rigid Body
     * Dynamics Algorithms" of Roy Featherstone, 2008
     * (ISBN:978-0-387-74314-1) See page 185 for the pseudo-code.
     * 
     * It calculates the torques for the joints and acceleration of the
     * floating base given the motion of the the joints (q,qdot,qdotdot),
     * the floating base frame pose and twist in fixed base coordinates, 
     * the external forces on the segments (expressed in the segments 
     * reference frame) and the dynamical parameters of the segments.
     */
    class TreeIdFbSolver_RNE : public TreeIdFbSolver{
    public:
        /**
         * Constructor for the solver, it will allocate all the necessary memory
         * \param tree The kinematic tree to calculate the inverse dynamics for, an internal copy will be made.
         * \param grav The gravity vector to use during the calculation.
         */
        TreeIdFbSolver_RNE(const Tree& tree,Vector grav);
        ~TreeIdFbSolver_RNE(){};
        
        /**
         * Calculate floating base inverse dynamics, from joint positions, velocity, acceleration, external forces
         * Input parameters;
         * \param q The current joint positions
         * \param q_dot The current joint velocities
         * \param q_dotdot The current joint accelerations
         * \param f_ext The external forces (no gravity) on the segments
         * \param X_fb The floating base frame in fixed base coordinates
         * \param v_fb The floating base twist in fixed base coordinates 
         * Output parameters:
         * \param a_fb The floating base accelartion in fixed base coordinates
         * \param torques the resulting torques for the joints
         */

        int CartToJnt(const JntArray &q, const JntArray &q_dot, const JntArray &q_dotdot, const Wrenches& f_ext, const Frame &X_fb, const Twist &v_fb, Twist &a_fb, JntArray &torques);

    private:
        int initSegments(const SegmentMap::const_iterator parent_it, int index = -1);
        Tree tree;
        unsigned int nj;
        unsigned int ns;
        std::vector<Segment> segment;
        std::vector<int> parent;
        std::vector<RigidBodyInertia> I;
        std::vector<Frame> X;
        std::vector<Twist> S;
        std::vector<Twist> v;
        std::vector<Twist> a;
        std::vector<Wrench> f;
        Twist ag;
    };
}

#endif
