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

#include "treecogsolver_recursive.hpp"
#include "kdl/frames_io.hpp"

// Mike Hopkins 2/19/2013

namespace KDL{


    TreeCoGSolver_recursive::TreeCoGSolver_recursive(const Tree& tree_):
        tree(tree_),nj(tree.getNrOfJoints()),ns(tree.getNrOfSegments())
    {
        parent.resize(ns);
        segment.resize(ns);
        pose.resize(ns);
        mass = 0;
        cog = Vector::Zero();
        initSegments(tree.getRootSegment());
    }

    int TreeCoGSolver_recursive::JntToCoG(const JntArray &q, Vector &cog)
    {
        //Check sizes when in debug mode
        if (q.rows() != nj)
            return -1;

        Segment root_segment = tree.getRootSegment()->second.segment;
        Frame root_pose = root_segment.pose(0);
        mass = root_segment.getInertia().getMass(); 
        cog = root_segment.getInertia().getCOG();

        //Sweep from root to leaf
        unsigned int j = 0;
        for(unsigned int i=0;i<ns;i++){
            double q_;
            if(segment[i].getJoint().getType() != Joint::None) {
                q_ = q(j);
                j++;
            } else
                q_ = 0.0;
            
            //Compute segment pose relative root 
            int pi = parent[i];
            if (pi == -1) {
              pose[i] = root_pose*segment[i].pose(q_);
            } else {
              pose[i] = pose[pi]*segment[i].pose(q_);
            }

            //Compute new center of gravity
            double segment_mass = segment[i].getInertia().getMass();
            Vector segment_cog = segment[i].getInertia().getCOG();
            cog = (mass*cog + segment_mass*(pose[i]*segment_cog))/(mass + segment_mass);
            mass += segment_mass;
	  //std::cout << "mass = " << mass << "\n cog = " << cog << std::endl;
        }
	return 0;
    }

    int TreeCoGSolver_recursive::initSegments(SegmentMap::const_iterator parent_it, int index)
    {
        int parent_index = index;
        for (unsigned int i = 0; i < parent_it->second.children.size(); i++)
        {
            SegmentMap::const_iterator child_it = parent_it->second.children[i];
            index++;
            parent[index] = parent_index;
            segment[index] = child_it->second.segment;
          //std::cout << index << " : " << child_it->first << " (parent : " << parent_it->first << ")\n";
	    index = initSegments(child_it, index);
        }
        return index;
    }
}//namespace
