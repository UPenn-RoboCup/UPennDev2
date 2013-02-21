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

#ifndef KDL_TREE_COG_SOLVER_RECURSIVE_HPP
#define KDL_TREE_COG_SOLVER_RECURSIVE_HPP

#include "treecogsolver.hpp"

namespace KDL{
    /**
     * \brief Recursive Tree Center of Gravity Solver 
     *
     * Calculates the tree center of gravity (CoG) relative the 
     * fixed base (root segment) given the current joint positions.
     */
    class TreeCoGSolver_recursive : public TreeCoGSolver{
    public:
        /**
         * Constructor for the solver, it will allocate all the necessary memory
         * \param tree The kinematic tree to calculate the center of gravity, an internal copy will be made.
         */
        TreeCoGSolver_recursive(const Tree& tree);
        ~TreeCoGSolver_recursive(){};

	/** 
	 * Calculate tree center of gravity (CoG) from joint positions
         *
	 * Input parameters;
	 * @param q input joint positions
	 * Output parameters:
	 * @param CoG center of gravity in fixed base (root segment) coordinates
	 * @return if < 0 something went wrong
	 */
        int JntToCoG(const JntArray &q, Vector &cog);

    private:
        int initSegments(const SegmentMap::const_iterator parent_it, int index = -1);
        Tree tree;
        unsigned int nj;
        unsigned int ns;
        std::vector<Segment> segment;
        std::vector<int> parent;
        std::vector<Frame> pose;
        double mass;
        Vector cog;
    };
}

#endif
