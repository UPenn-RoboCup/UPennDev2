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

#include "treeidsolver_recursive_newton_euler.hpp"
#include "kdl/frames_io.hpp"

namespace KDL{


    TreeIdSolver_RNE::TreeIdSolver_RNE(const Tree& tree_,Vector grav):
        tree(tree_),nj(tree.getNrOfJoints()),ns(tree.getNrOfSegments())
    {
        parent.resize(ns);
        segment.resize(ns);
        X.resize(ns);
        S.resize(ns);
        v.resize(ns);
        a.resize(ns);
        f.resize(ns);
        ag=-Twist(grav,Vector::Zero());
        initSegments(tree.getRootSegment());
    }

    int TreeIdSolver_RNE::CartToJnt(const JntArray &q, const JntArray &q_dot, const JntArray &q_dotdot, const Wrenches& f_ext, JntArray &torques)
    {
        //Check sizes when in debug mode
        if(q.rows()!=nj || q_dot.rows()!=nj || q_dotdot.rows()!=nj || torques.rows()!=nj || f_ext.size()!=ns)
            return -1;
        unsigned int j=0;

        //Sweep from root to leaf
        for(unsigned int i=0;i<ns;i++){
            double q_,qdot_,qdotdot_;
            if(segment[i].getJoint().getType()!=Joint::None){
                q_=q(j);
                qdot_=q_dot(j);
                qdotdot_=q_dotdot(j);
                j++;
            }else
                q_=qdot_=qdotdot_=0.0;
            
            //Calculate segment properties: X,S,vj,cj
            X[i]=segment[i].pose(q_);//Remark this is the inverse of the 
                                                //frame for transformations from 
                                                //the parent to the current coord frame
            //Transform velocity and unit velocity to segment frame
            Twist vj=X[i].M.Inverse(segment[i].twist(q_,qdot_));
            S[i]=X[i].M.Inverse(segment[i].twist(q_,1.0));
            //We can take cj=0, see remark section 3.5, page 55 since the unit velocity vector S of our joints is always time constant
            //calculate velocity and acceleration of the segment (in segment coordinates)
            int pi = parent[i];
            if(pi==-1){
                v[i]=vj;
                a[i]=X[i].Inverse(ag)+S[i]*qdotdot_+v[i]*vj;
            }else{
                v[i]=X[i].Inverse(v[pi])+vj;
                a[i]=X[i].Inverse(a[pi])+S[i]*qdotdot_+v[i]*vj;
            }
            //Calculate the force for the joint
            //Collect RigidBodyInertia and external forces
            RigidBodyInertia Ii=segment[i].getInertia();
            f[i]=Ii*a[i]+v[i]*(Ii*v[i])-f_ext[i];
	  //std::cout << "a[i]=" << a[i] << "\n f[i]=" << f[i] << "\n S[i]" << S[i] << std::endl;
        }
        //Sweep from leaf to root
        j=nj-1;
        for(int i=ns-1;i>=0;i--){
            if(segment[i].getJoint().getType()!=Joint::None)
                torques(j--)=dot(S[i],f[i]);

            int pi = parent[i];
            if(pi!=-1)
                f[pi]=f[pi]+X[i]*f[i];
        }
	return 0;
    }

    int TreeIdSolver_RNE::initSegments(SegmentMap::const_iterator parent_it, int index)
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
