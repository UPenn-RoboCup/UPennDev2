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

#include "treeidfbsolver_recursive_newton_euler.hpp"
#include "kdl/frames_io.hpp"
#include "pluckermatrix.hpp"
#include "spatialvector.hpp"

// Mike Hopkins 2/17/2013

namespace KDL{


    TreeIdFbSolver_RNE::TreeIdFbSolver_RNE(const Tree& tree_,Vector grav):
        tree(tree_),nj(tree.getNrOfJoints()),ns(tree.getNrOfSegments())
    {
        parent.resize(ns);
        segment.resize(ns);
        I.resize(ns);
        X.resize(ns);
        S.resize(ns);
        v.resize(ns);
        a.resize(ns);
        f.resize(ns);
        ag=-Twist(grav,Vector::Zero());
        initSegments(tree.getRootSegment());
    }

    int TreeIdFbSolver_RNE::CartToJnt(const JntArray &q, const JntArray &q_dot, const JntArray &q_dotdot, const Wrenches& f_ext, const Frame &X_fb, const Twist &v_fb, Twist &a_fb, JntArray &torques)
    {
        //Check sizes when in debug mode
        if(q.rows()!=nj || q_dot.rows()!=nj || q_dotdot.rows()!=nj || torques.rows()!=nj || f_ext.size()!=ns)
            return -1;

        //Compute initial floating base velocity / acceleration
        //The root segment serves as the floating base
        //Note X_fb is the inverse of the transform
        //from fixed to floating based coordinates
        RigidBodyInertia Ir=tree.getRootSegment()->second.segment.getInertia();
        Twist vr=X_fb.Inverse(v_fb);
        Twist ar=X_fb.Inverse(ag);
        Wrench fr=Ir*ar+vr*(Ir*vr);

        //Sweep from root to leaf
        unsigned int j=0;
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
            int pi=parent[i];
            if(pi==-1){
	      v[i]=X[i].Inverse(vr)+vj;
	      a[i]=X[i].Inverse(ar)+S[i]*qdotdot_+v[i]*vj;
            }else{
	      v[i]=X[i].Inverse(v[pi])+vj;
	      a[i]=X[i].Inverse(a[pi])+S[i]*qdotdot_+v[i]*vj;
            }

            //Calculate the force for the joint
            //Collect RigidBodyInertia and external forces
            I[i]=segment[i].getInertia();
            f[i]=I[i]*a[i]+v[i]*(I[i]*v[i])-f_ext[i];
	  //std::cout << "a[i]=" << a[i] << "\n f[i]=" << f[i] << "\n S[i]" << S[i] << std::endl;
        }

        //Sweep from leaf to root
        for(int i=ns-1;i>=0;i--){
            PluckerMatrix Xi=PluckerMatrix(X[i].Inverse());
            PluckerMatrix Ii=PluckerMatrix(I[i]);
            int pi=parent[i];
            if(pi==-1){
	      Ir=Ir+(Xi.Transpose()*Ii*Xi).toRigidBodyInertia();
	      fr=fr+X[i]*f[i]; 
            }else{
	      I[pi]=I[pi]+(Xi.Transpose()*Ii*Xi).toRigidBodyInertia();
	      f[pi]=f[pi]+X[i]*f[i]; 
            }
        }

        //Compute true floating base acceleration
        ar=-(PluckerMatrix(Ir).Inverse()*SpatialVector(fr)).toTwist();

        //Sweep from root to leaf
        j=0;
        for(int i=0;i<ns;i++){
            int pi=parent[i];
            if(pi==-1){
              a[i]=X[i].Inverse(ar);
            }else{
              a[i]=X[i].Inverse(a[pi]);
            }
            if(segment[i].getJoint().getType()!=Joint::None)
                torques(j++)=dot(S[i],I[i]*a[i]+f[i]);
        }

        //Transform floating base acceleration to fixed base frame
        a_fb=X_fb*ar;

	return 0;
    }

    int TreeIdFbSolver_RNE::initSegments(SegmentMap::const_iterator parent_it, int index)
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
