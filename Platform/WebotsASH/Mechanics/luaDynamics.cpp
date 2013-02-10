#include "luaKDL_utils.h"
#include "luaDynamics.h"
#include "Mechanics.h"
#include <stdio.h>

using namespace KDL;

extern _Mechanics Mechanics;

static int lua_inverse(lua_State *L)
{
  // TODO
  int n_joints = Mechanics.body.getNrOfJoints();
  int n_segments = Mechanics.body.getNrOfSegments();
  printf("n_joints %i\n", n_joints);
  printf("n_segments %i\n", n_segments);

/*
  JntArray torques(n_joints);
  Wrenches f_ext(n_segments);
  JntArray q = lua_checkJntArray(L, 1);
  JntArray qdot = lua_checkJntArray(L, 2);
  JntArray qdotdot = lua_checkJntArray(L, 3);
  JntArray torso_pos(6);
  JntArray torso_vel(6);
  JntArray torso_acc(6);

  if (lua_istable(L, 4))
    torso_pos = lua_checkJntArray(L, 4);
  if (lua_istable(L, 5))
    torso_vel = lua_checkJntArray(L, 5);
  if (lua_istable(L, 6))
    torso_acc = lua_checkJntArray(L, 6);

  q.resize(n_joints);
  qdot.resize(n_joints);
  qdotdot.resize(n_joints);
  torso_pos.resize(6);
  torso_vel.resize(6);
  torso_acc.resize(6);

  int success = Mechanics.body_id_solver->JntToCart(
    q,
    qdot,
    qdotdot,
    f_ext,
    torques
  );
  assert(success);
*/
  return 0;
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
