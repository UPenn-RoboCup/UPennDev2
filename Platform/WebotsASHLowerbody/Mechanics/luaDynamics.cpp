#include "luaKDL_utils.h"
#include "luaDynamics.h"
#include "Mechanics.h"
#include <stdio.h>
#include <iostream>

using namespace KDL;

extern _Mechanics Mechanics;

static int lua_inverse(lua_State *L)
{
  // Compute full body floating base inverse dynamics using RNE
  int n_joints = Mechanics.body.getNrOfJoints();
  int n_segments = Mechanics.body.getNrOfSegments();

  JntArray torso_pos(6);
  JntArray torso_vel(6);
  JntArray torso_acc(6);
  JntArray torques(n_joints);
  Wrenches f_ext(n_segments);

  // Get joint positions, velocities and accelerations
  JntArray q_in = lua_checkJntArray(L, 1);
  JntArray qdot_in = lua_checkJntArray(L, 2);
  JntArray qdotdot_in = lua_checkJntArray(L, 3);

  luaL_argcheck(L, (q_in.rows() == n_joints - 6), 1, "dimension error");
  luaL_argcheck(L, (qdot_in.rows() == n_joints - 6), 2, "dimension error");
  luaL_argcheck(L, (qdotdot_in.rows() == n_joints - 6), 3, "dimension error");

  // Get optional 6 DOF position, velocity and acceleration of torso
  if (lua_istable(L, 4))
    torso_pos = lua_checkJntArray(L, 4);
  if (lua_istable(L, 5))
    torso_vel = lua_checkJntArray(L, 5);
  if (lua_istable(L, 6))
    torso_acc = lua_checkJntArray(L, 6);

  luaL_argcheck(L, (torso_pos.rows() == 6), 4, "dimension error");
  luaL_argcheck(L, (torso_vel.rows() == 6), 5, "dimension error");
  luaL_argcheck(L, (torso_acc.rows() == 6), 6, "dimension error");

  // Get optional contact wrenches at each end-effector
  for (int i = 0; i < n_joints; i++)
    f_ext[i] = Wrench::Zero();
  if (lua_istable(L, 7))
    f_ext[Mechanics.body_l_foot_index] = lua_checkWrench(L, 7);
  if (lua_istable(L, 8))
    f_ext[Mechanics.body_r_foot_index] = lua_checkWrench(L, 8);
  if (lua_istable(L, 9))
    f_ext[Mechanics.body_l_hand_index] = lua_checkWrench(L, 9);
  if (lua_istable(L, 10))
    f_ext[Mechanics.body_r_hand_index] = lua_checkWrench(L, 10);

  // Add 6 DOF floating base joint to account for torso motion
  JntArray q(n_joints);
  JntArray qdot(n_joints);
  JntArray qdotdot(n_joints);

  for (int i = 0; i < 6; i++)
    q(i) = torso_pos(i);
  for (int i = 0; i < 6; i++)
    qdot(i) = torso_vel(i);
  for (int i = 0; i < 6; i++)
    qdotdot(i) = torso_acc(i);
  for (int i = 6; i < n_joints; i++)
    q(i) = q_in(i - 6);
  for (int i = 6; i < n_joints; i++)
    qdot(i) = qdot_in(i - 6);
  for (int i = 6; i < n_joints; i++)
    qdotdot(i) = qdotdot_in(i - 6);

  // Solve for joint torques using Recursive Newton Euler method
  int status = Mechanics.body_id_solver->CartToJnt(
    q,
    qdot,
    qdotdot,
    f_ext,
    torques
  );
  if (status != 0)
    luaL_error(L, "inverse dynamics solver failed");

  // Return joint torques
  JntArray joint_torques(n_joints - 6);
  for (int i = 6; i < n_joints; i++)
    joint_torques(i - 6) = torques(i);
  lua_pushJntArray(L, joint_torques);

  // Return 6 DOF floating base torques (for debugging)
  JntArray floating_base_torques(6);
  for (int i = 0; i < 6; i++)
    floating_base_torques(i) = torques(i);
  lua_pushJntArray(L, floating_base_torques);
  return 2;
}

static int lua_cog(lua_State *L)
{
  // TODO
  return 0;
}

static const struct luaL_reg Dynamics_lib [] = {
  {"inverse", lua_inverse},
  {"cog", lua_cog}
};

extern "C"
int luaopen_Dynamics(lua_State *L)
{
  luaL_register(L, "Dynamics", Dynamics_lib);
  return 1;
}
