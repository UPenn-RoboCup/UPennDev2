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

///  This and the source file were modified from ChainIdSolver_recursive_newton_euler.h/cpp
///  Thus, still probably covered by LGPL, which is why I build it as a library and then link to it
///     Darren Earl, HRL 6/2012



#include "treeidsolver_recursive_newton_euler.hpp"
#include "kdl/frames_io.hpp"

#include <deque>
using namespace std;

namespace KDL{
    
    TreeIdSolver_RNE::TreeIdSolver_RNE(const Tree& tree_,Vector grav)
    :tree(tree_)
    {
		root_name = tree.getRootSegment()->first;
		
        ag=-Twist(grav,Vector::Zero());
        const SegmentMap& sm = tree.getSegments();
        
        int jnt =0;
        for( SegmentMap::const_iterator i=sm.begin(); i!=sm.end(); ++i ){
			const Segment& s = i->second.segment;
			const Joint& j = s.getJoint();
			if( j.getType() != Joint::None ){
				JntEntry& je = jntdb[ j.getName() ];
				je.idx = jnt++;
			}
				
				
			const string& name = i->first;
			db[ name ]; // create the element here, but don't have anything to fill in
			/*
			X[ name ];
			S[ name ];
			v[ name ];
			a[ name ];
			f[ name ];
			f_ext_map[ name ];
			*/
		}
    }

    int TreeIdSolver_RNE::CartToJnt(const std::vector<double> &q, const std::vector<double> &q_dot, const std::vector<double> &q_dotdot, const Wrenches& f_ext,JntArray &torques)
    {
        //Check sizes when in debug mode
        //if(q.rows()!=nj || q_dot.rows()!=nj || q_dotdot.rows()!=nj || torques.rows()!=nj || f_ext.size()!=ns)
         //   return -1;
        
        {
			const SegmentMap& sm = tree.getSegments();
			int cnt =0;
			for( SegmentMap::const_iterator i=sm.begin(); i!=sm.end(); ++i ){
				Entry& e = db[ i->first ];
				e.f_ext = f_ext[ cnt++ ];
			}
		}
        
        
        ///these are not RT safe...
        /// maps are dangerous because might create element inadvertently
        deque< SegmentMap::const_iterator > open;
        vector< SegmentMap::const_iterator > processed; 
        open.push_back( tree.getRootSegment() );

        //Sweep from root to leaf
        while( !open.empty() ){
			SegmentMap::const_iterator te = open.front();
			open.pop_front();
			processed.push_back( te );
			for( std::vector< SegmentMap::const_iterator >::const_iterator i=te->second.children.begin();
			     i != te->second.children.end();
			     ++i )
			{
				open.push_back( *i );
			}
			
			
			const Segment& seg = te->second.segment;
			const Joint& jnt = seg.getJoint();
			
            double q_,qdot_,qdotdot_;
            if(jnt.getType() !=Joint::None){
				
				int idx = jntdb[ jnt.getName() ].idx;
                q_=q[idx];
                qdot_=q_dot[idx];
                qdotdot_=q_dotdot[idx];
            }else
                q_=qdot_=qdotdot_=0.0;
                
            Entry& e = db[seg.getName()];
                
            Frame& eX  = e.X;
            Twist& eS  = e.S;
            Twist& ev  = e.v;
            Twist& ea  = e.a;
            Wrench& ef = e.f;
            Wrench& ef_ext = e.f_ext;
            
            string parent_name = "fake";
            if( te->first != root_name )
				parent_name = te->second.parent->first;
            
            Entry& parent_entry = db[parent_name];
            Twist& parent_a = parent_entry.a;
            Twist& parent_v = parent_entry.v;
            
            //Calculate segment properties: X,S,vj,cj
            eX=seg.pose(q_);//Remark this is the inverse of the 
                            //frame for transformations from 
                            //the parent to the current coord frame
            //Transform velocity and unit velocity to segment frame
            Twist vj=eX.M.Inverse(seg.twist(q_,qdot_));
            eS=eX.M.Inverse(seg.twist(q_,1.0));
            //We can take cj=0, see remark section 3.5, page 55 since the unit velocity vector S of our joints is always time constant
            //calculate velocity and acceleration of the segment (in segment coordinates)
            if( te->first == root_name ){
                ev=vj;
                ea=eX.Inverse(ag)+eS*qdotdot_+ev*vj;
            }else{
                ev=eX.Inverse(parent_v)+vj;
                ea=eX.Inverse(parent_a)+eS*qdotdot_+ev*vj;
            }
            //Calculate the force for the joint
            //Collect RigidBodyInertia and external forces
            RigidBodyInertia Ii=seg.getInertia();
            ef=Ii*ea+ev*(Ii*ev)-ef_ext;
	    //std::cout << "a[i]=" << a[i] << "\n f[i]=" << f[i] << "\n S[i]" << S[i] << std::endl;
        }
        // process processed back to front...        
        //Sweep from leaf to root
        int size = processed.size()-1;
        for(int i=size; i>0; i--){
			SegmentMap::const_iterator te = processed[i];
			const Segment& seg = te->second.segment;
			const Joint& jnt = seg.getJoint();
			Entry& e = db[seg.getName()];
			Frame& eX = e.X;
            Twist& eS = e.S;
            Wrench& ef = e.f;
            string parent_name = te->second.parent->first;
			Entry& parent_e = db[parent_name];
			Wrench& pre_f = parent_e.f;
			
			
            if(jnt.getType()!=Joint::None)
                jntdb[jnt.getName()].torque=dot(eS,ef);
            pre_f +=eX*ef;
        }
        // do root element
        // probably could hard code this lookup to make a little bit faster
        { 
			const TreeElement& te = tree.getRootSegment()->second;
			const Segment& seg = te.segment;
			const Joint& jnt = seg.getJoint();
			Entry& e = db[root_name];
			if( jnt.getType() != Joint::None )
				jntdb[jnt.getName()].torque = dot( e.S, e.f );
		}
		for( map<string, JntEntry>::const_iterator i= jntdb.begin(); i!= jntdb.end(); ++i ){
			torques(i->second.idx) = i->second.torque;
		}
	return 0;
    }

    int TreeIdSolver_RNE::CartToJnt(const JntArray &q, const JntArray &q_dot, const JntArray &q_dotdot, const Wrenches& f_ext,JntArray &torques)
    {
        //Check sizes when in debug mode
        if(q.rows()!=nj || q_dot.rows()!=nj || q_dotdot.rows()!=nj || torques.rows()!=nj || f_ext.size()!=ns)
            return -1;
        {
			const SegmentMap& sm = tree.getSegments();
			int cnt =0;
			for( SegmentMap::const_iterator i=sm.begin(); i!=sm.end(); ++i ){
				Entry& e = db[ i->first ];
				e.f_ext = f_ext[ cnt++ ];
			}
		}
        
        ///these are not RT safe...
        /// maps are dangerous because might create element inadvertently
        deque< SegmentMap::const_iterator > open;
        vector< SegmentMap::const_iterator > processed; 
        open.push_back( tree.getRootSegment() );

        //Sweep from root to leaf
        while( !open.empty() ){
			SegmentMap::const_iterator te = open.front();
			open.pop_front();
			processed.push_back( te );
			for( std::vector< SegmentMap::const_iterator >::const_iterator i=te->second.children.begin();
			     i != te->second.children.end();
			     ++i )
			{
				open.push_back( *i );
			}
			
			
			const Segment& seg = te->second.segment;
			const Joint& jnt = seg.getJoint();
			
            double q_,qdot_,qdotdot_;
            if(jnt.getType() !=Joint::None){
				
				int idx = jntdb[ jnt.getName() ].idx;
                q_=q(idx);
                qdot_=q_dot(idx);
                qdotdot_=q_dotdot(idx);
            }else
                q_=qdot_=qdotdot_=0.0;
                
            Entry& e = db[seg.getName()];
                
            Frame& eX  = e.X;
            Twist& eS  = e.S;
            Twist& ev  = e.v;
            Twist& ea  = e.a;
            Wrench& ef = e.f;
            Wrench& ef_ext = e.f_ext;
            
            string parent_name = "fake";
            if( te->first != root_name )
				parent_name = te->second.parent->first;
            
            Entry& parent_entry = db[parent_name];
            Twist& parent_a = parent_entry.a;
            Twist& parent_v = parent_entry.v;
            
            //Calculate segment properties: X,S,vj,cj
            eX=seg.pose(q_);//Remark this is the inverse of the 
                            //frame for transformations from 
                            //the parent to the current coord frame
            //Transform velocity and unit velocity to segment frame
            Twist vj=eX.M.Inverse(seg.twist(q_,qdot_));
            eS=eX.M.Inverse(seg.twist(q_,1.0));
            //We can take cj=0, see remark section 3.5, page 55 since the unit velocity vector S of our joints is always time constant
            //calculate velocity and acceleration of the segment (in segment coordinates)
            if( te->first == root_name ){
                ev=vj;
                ea=eX.Inverse(ag)+eS*qdotdot_+ev*vj;
            }else{
                ev=eX.Inverse(parent_v)+vj;
                ea=eX.Inverse(parent_a)+eS*qdotdot_+ev*vj;
            }
            //Calculate the force for the joint
            //Collect RigidBodyInertia and external forces
            RigidBodyInertia Ii=seg.getInertia();
            ef=Ii*ea+ev*(Ii*ev)-ef_ext;
	    //std::cout << "a[i]=" << a[i] << "\n f[i]=" << f[i] << "\n S[i]" << S[i] << std::endl;
        }
        // process processed back to front...        
        //Sweep from leaf to root
        int size = processed.size()-1;
        for(int i=size; i>0; i--){
			SegmentMap::const_iterator te = processed[i];
			const Segment& seg = te->second.segment;
			const Joint& jnt = seg.getJoint();
			Entry& e = db[seg.getName()];
			Frame& eX = e.X;
            Twist& eS = e.S;
            Wrench& ef = e.f;
            string parent_name = te->second.parent->first;
			Entry& parent_e = db[parent_name];
			Wrench& pre_f = parent_e.f;
			
			
            if(jnt.getType()!=Joint::None)
                jntdb[jnt.getName()].torque=dot(eS,ef);
            pre_f +=eX*ef;
        }
        // do root element
        // probably could hard code this lookup to make a little bit faster
        { 
			const TreeElement& te = tree.getRootSegment()->second;
			const Segment& seg = te.segment;
			const Joint& jnt = seg.getJoint();
			Entry& e = db[root_name];
			if( jnt.getType() != Joint::None )
				jntdb[jnt.getName()].torque = dot( e.S, e.f );
		}
		for( map<string, JntEntry>::const_iterator i= jntdb.begin(); i!= jntdb.end(); ++i ){
			torques(i->second.idx) = i->second.torque;
		}
	return 0;
    }


}//namespace
