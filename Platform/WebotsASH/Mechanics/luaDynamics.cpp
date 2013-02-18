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

  Frame torso_frame;
  Twist torso_twist;
  Twist torso_accel;
  JntArray torques(n_joints);
  Wrenches f_ext(n_segments);

  // Get joint positions, velocities and accelerations
  JntArray q_in = lua_checkJntArray(L, 1);
  JntArray qdot_in = lua_checkJntArray(L, 2);
  JntArray qdotdot_in = lua_checkJntArray(L, 3);

  luaL_argcheck(L, (q_in.rows() >= n_joints), 1, "dimension error");
  luaL_argcheck(L, (qdot_in.rows() >= n_joints), 2, "dimension error");
  luaL_argcheck(L, (qdotdot_in.rows() >= n_joints), 3, "dimension error");

  JntArray q(n_joints);
  JntArray qdot(n_joints);
  JntArray qdotdot(n_joints);

  for (int i = 0; i < n_joints; i++) {
    q(i) = q_in(i);
    qdot(i) = qdot_in(i);
    qdotdot(i) = qdotdot_in(i);
  }

  // Get optional position and velocity of torso relative fixed base
  if (lua_istable(L, 4))
    torso_frame = lua_checkFrame(L, 4);
  if (lua_istable(L, 5))
    torso_twist = lua_checkTwist(L, 5);

  // Get optional contact wrenches at each end-effector
  for (int i = 0; i < n_joints; i++)
    f_ext[i] = Wrench::Zero();
  if (lua_istable(L, 6))
    f_ext[Mechanics.body_l_foot_index] = lua_checkWrench(L, 6);
  if (lua_istable(L, 7))
    f_ext[Mechanics.body_r_foot_index] = lua_checkWrench(L, 7);
  if (lua_istable(L, 8))
    f_ext[Mechanics.body_l_hand_index] = lua_checkWrench(L, 8);
  if (lua_istable(L, 9))
    f_ext[Mechanics.body_r_hand_index] = lua_checkWrench(L, 9);

  // Solve for joint torques using Recursive Newton Euler method
  int status = Mechanics.body_id_solver->CartToJnt(
    q,
    qdot,
    qdotdot,
    f_ext,
    torso_frame,
    torso_twist,
    torso_accel,
    torques
  );
  if (status != 0)
    luaL_error(L, "inverse dynamics solver failed");

  // Return joint torques
  lua_pushJntArray(L, torques);

  // Return torso acceleration
  lua_pushTwist(L, torso_accel);
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
