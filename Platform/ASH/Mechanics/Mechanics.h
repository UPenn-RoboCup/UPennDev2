#ifndef _Mechanics_H_
#define _Mechanics_H_

#include <kdl/chain.hpp>
#include <kdl/chainiksolver.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>
#include <kdl/chaindynparam.hpp>
#include "treeidsolver.hpp"
#include "treeidsolver_recursive_newton_euler.hpp"

using namespace KDL;

typedef struct _Mechanics {

  Vector gravity;

  // coordinate transforms
  /////////////////////////////////////////////////////////////////////////

  // head
  Frame clavicle_transform;
  Frame neck_transform;
  Frame head_transform;

  // l_arm
  Frame l_shoulder_transform;
  Frame l_arm_transform;
  Frame l_forearm_transform;
  Frame l_hand_transform;

  // r_arm 
  Frame r_shoulder_transform;
  Frame r_arm_transform;
  Frame r_forearm_transform;
  Frame r_hand_transform;

  // waist
  Frame torso_waist_transform;
  Frame waist_chest_transform;

  // l_leg
  Frame l_hip_transform;
  Frame l_thigh_transform;
  Frame l_shin_transform;
  Frame l_foot_transform;

  // r_leg
  Frame r_hip_transform;
  Frame r_thigh_transform;
  Frame r_shin_transform;
  Frame r_foot_transform;

  // link inertias
  /////////////////////////////////////////////////////////////////////////

  // head
  RigidBodyInertia neck_inertia;
  RigidBodyInertia head_inertia;

  // l_arm
  RigidBodyInertia l_arm_inertia;
  RigidBodyInertia l_forearm_inertia;
  RigidBodyInertia l_hand_inertia;

  // r_arm
  RigidBodyInertia r_arm_inertia;
  RigidBodyInertia r_forearm_inertia;
  RigidBodyInertia r_hand_inertia;

  // waist
  RigidBodyInertia torso_inertia;
  RigidBodyInertia chest_inertia;

  // l_leg
  RigidBodyInertia l_thigh_inertia;
  RigidBodyInertia l_shin_inertia;
  RigidBodyInertia l_foot_inertia;

  // r_leg
  RigidBodyInertia r_thigh_inertia;
  RigidBodyInertia r_shin_inertia;
  RigidBodyInertia r_foot_inertia;

  // kinematic chains
  /////////////////////////////////////////////////////////////////////////

  Chain head;
  Chain l_arm;
  Chain r_arm;
  Chain waist;
  Chain l_leg;
  Chain r_leg;
  Chain base;

  // kinematic trees
  /////////////////////////////////////////////////////////////////////////
  Tree body;

  // kinematic and dynamic solvers
  /////////////////////////////////////////////////////////////////////////

  // head
  ChainFkSolverPos_recursive *head_fk_pos_solver;
  ChainFkSolverVel_recursive *head_fk_vel_solver;
  ChainIkSolverVel_pinv *head_ik_vel_solver;
  ChainJntToJacSolver *head_jnt_to_jac_solver;
  ChainDynParam *head_dynamic_param;

  // l_arm
  ChainFkSolverPos_recursive *l_arm_fk_pos_solver;
  ChainFkSolverVel_recursive *l_arm_fk_vel_solver;
  ChainIkSolverVel_pinv *l_arm_ik_vel_solver;
  ChainJntToJacSolver *l_arm_jnt_to_jac_solver;
  ChainDynParam *l_arm_dynamic_param;

  // r_arm
  ChainFkSolverPos_recursive *r_arm_fk_pos_solver;
  ChainFkSolverVel_recursive *r_arm_fk_vel_solver;
  ChainIkSolverVel_pinv *r_arm_ik_vel_solver;
  ChainJntToJacSolver *r_arm_jnt_to_jac_solver;
  ChainDynParam *r_arm_dynamic_param;

  // waist
  ChainFkSolverPos_recursive *waist_fk_pos_solver;
  ChainFkSolverVel_recursive *waist_fk_vel_solver;
  ChainIkSolverVel_pinv *waist_ik_vel_solver;
  ChainJntToJacSolver *waist_jnt_to_jac_solver;
  ChainDynParam *waist_dynamic_param;

  // l_leg
  ChainFkSolverPos_recursive *l_leg_fk_pos_solver;
  ChainFkSolverVel_recursive *l_leg_fk_vel_solver;
  ChainIkSolverVel_pinv *l_leg_ik_vel_solver;
  ChainJntToJacSolver *l_leg_jnt_to_jac_solver;
  ChainDynParam *l_leg_dynamic_param;

  // r_leg
  ChainFkSolverPos_recursive *r_leg_fk_pos_solver;
  ChainFkSolverVel_recursive *r_leg_fk_vel_solver;
  ChainIkSolverVel_pinv *r_leg_ik_vel_solver;
  ChainJntToJacSolver *r_leg_jnt_to_jac_solver;
  ChainDynParam *r_leg_dynamic_param;

  // body
  TreeIdSolver *body_id_solver;

  _Mechanics();
} _Mechanics;

#endif
