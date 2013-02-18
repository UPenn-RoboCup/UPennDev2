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

#include "kdl/tree.hpp"
#include "kdl/chain.hpp"
#include "kdl/frames.hpp"
#include "kdl/jntarray.hpp"

namespace KDL
{

    typedef std::vector<Wrench> Wrenches;

	/**
	 * \brief This <strong>abstract</strong> class encapsulates the floating
	 * base inverse dynamics solver for a KDL::Tree.
	 *
	 */
	class TreeIdFbSolver
	{
		public:
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
		 * \param torques the resulting torques for the joints
		 * \param a_fb The floating base accelartion in fixed base coordinates
		 */
        virtual int CartToJnt(const JntArray &q, const JntArray &q_dot, const JntArray &q_dotdot, const Wrenches& f_ext, const Frame &X_fb, const Twist &v_fb, Twist &a_fb, JntArray &torques)=0;

        // Need functions to return the manipulator mass, coriolis and gravity matrices - Lagrangian Formulation.
	};

}
