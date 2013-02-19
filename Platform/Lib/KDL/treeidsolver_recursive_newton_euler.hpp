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

#pragma once

#include "treeidsolver.hpp"

namespace KDL{
    /**
     * \brief Recursive newton euler inverse dynamics solver
     *
     * The algorithm implementation is based on the book "Rigid Body
     * Dynamics Algorithms" of Roy Featherstone, 2008
     * (ISBN:978-0-387-74314-1) See page 96 for the pseudo-code.
     * 
     * It calculates the torques for the joints, given the motion of
     * the joints (q,qdot,qdotdot), external forces on the segments
     * (expressed in the segments reference frame) and the dynamical
     * parameters of the segments.
     */
    class TreeIdSolver_RNE : public TreeIdSolver{
    public:
        /**
         * Constructor for the solver, it will allocate all the necessary memory
         * \param chain The kinematic chain to calculate the inverse dynamics for, an internal copy will be made.
         * \param grav The gravity vector to use during the calculation.
         */
        TreeIdSolver_RNE(const Tree& tree,Vector grav);
        ~TreeIdSolver_RNE(){};
        
        /**
         * Function to calculate from Cartesian forces to joint torques.
         * Input parameters;
         * \param q The current joint positions
         * \param q_dot The current joint velocities
         * \param q_dotdot The current joint accelerations
         * \param f_ext The external forces (no gravity) on the segments
         * Output parameters:
         * \param torques the resulting torques for the joints
         */
        int CartToJnt(const JntArray &q, const JntArray &q_dot, const JntArray &q_dotdot, const Wrenches& f_ext,JntArray &torques);
        int CartToJnt(const std::vector<double> &q, const std::vector<double> &q_dot, const std::vector<double> &q_dotdot, const Wrenches& f_ext,JntArray &torques);


    private:
		struct Entry{
			Frame X;
			Twist S;
			Twist v;
			Twist a;
			Wrench f;
			Wrench f_ext;
		};
		struct JntEntry{
			int idx;
			double torque;
		};
			
    
    
    
        Tree tree;
        unsigned int nj;
        unsigned int ns;
        std::string root_name;
        //std::map<std::string, Frame> X;
        //std::map<std::string, Twist> S;
        //std::map<std::string, Twist> v;
        //std::map<std::string, Twist> a;
        //std::map<std::string, Wrench> f;
        //std::map<std::string, Wrench> f_ext_map;
        //much faster not to have to look up each individual element
        //better to group the variables together and only have one lookup
        //before was nearly 25% to look up each individually
        std::map<std::string, Entry> db;	///indexed by segment name
        std::map<std::string, JntEntry> jntdb;/// indexed by joint name
        
        //std::map<std::string, double> torque_map; /// takes in joint names
        //std::map<std::string, int> name2idx;
        Twist ag;
    };
}
