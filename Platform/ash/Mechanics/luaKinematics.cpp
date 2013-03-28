#include "luaKDL_utils.h"
#include "luaKinematics.h"
#include "Mechanics.h"

using namespace KDL;

extern _Mechanics Mechanics;

// platform specific IK position solvers
//////////////////////////////////////////////////////////////////////////

static void head_ik_pos_solver(const Vector &p_gaze, JntArray &q_head)
{
  // TODO
}

static void arm_ik_pos_solver(const Chain &arm, const Frame &p_hand,
                              JntArray &q_arm)
{
  // TODO
}

static void leg_ik_pos_solver(const Chain &leg, const Frame &p_foot, 
                                JntArray &q_leg)
{
  // solves inverse position kinematics for left or right leg
  // * assumptions :
  // * joint order is hip_z, hip_x, hip_y, knee_y, ankle_y, ankle_x
  // * hip_z, hip_x and hip_y axes are collocated
  // * ankle_y and ankle_x axes are collocated
  // * shin x offset and y offset are zero
  // * thigh x offset is nonnegative and y offset is zero

  // get coordinate transformations
  Frame torso_transform = leg.getSegment(0).getFrameToTip();
  Frame thigh_transform = leg.getSegment(3).getFrameToTip();
  Frame shin_transform = leg.getSegment(4).getFrameToTip();
  Frame foot_transform = leg.getSegment(6).getFrameToTip();

  // get link parameters
  Vector v_thigh = thigh_transform.p;
  Vector v_shin = shin_transform.p;
  double d_thigh = v_thigh.Norm(); 
  double d_shin = v_shin.Norm(); 
  double a_thigh = atan2(v_thigh(0), -v_thigh(2));

  // get hip coordinates relative to ankle frame
  Frame p_leg = foot_transform*p_foot.Inverse()*torso_transform;
  Vector v_leg = p_leg.p;
  double d_leg = v_leg.Norm();

  // get nominal knee position using law of cosines
  double c_knee = (d_leg*d_leg - d_thigh*d_thigh - d_shin*d_shin)
                / (2*d_thigh*d_shin);
  if (c_knee > 1) c_knee = 1;
  if (c_knee <-1) c_knee =-1;
  double knee_pitch = acos(c_knee);

  // get nominal ankle positions using law of sines
  double ankle_pitch = asin(-v_leg(0)/d_leg)
                     - asin(d_thigh*sin(knee_pitch)/d_leg);
  double ankle_roll = atan2(v_leg(1), v_leg(2));

  // get nominal hip positions by inverting ankle and pitch rotations
  Frame M_hip = p_foot
              * Frame(Rotation::RotX(-ankle_roll)
              * Rotation::RotY(-knee_pitch - ankle_pitch));
  double hip_yaw = atan2(-M_hip(0,1), M_hip(1,1));
  double hip_roll = asin(M_hip(2,1));
  double hip_pitch = atan2(-M_hip(2,0), M_hip(2,2));

  // adjust hip and knee pitch to compensate for thigh x offset
  q_leg(0) = hip_yaw;
  q_leg(1) = hip_roll;
  q_leg(2) = hip_pitch + a_thigh;
  q_leg(3) = knee_pitch - a_thigh;
  q_leg(4) = ankle_pitch;
  q_leg(5) = ankle_roll;
}

// forward position kinematics
///////////////////////////////////////////////////////////////////////////

static int lua_forward_pos_head(lua_State *L)
{
  // returns head frame relative to chest frame
  Frame p_head;
  JntArray q_head = lua_checkJntArray(L, 1);
  assert(!Mechanics.head_fk_pos_solver->JntToCart(q_head, p_head));
  lua_pushFrame(L, p_head);
  return 1;
}

static int lua_forward_pos_l_arm(lua_State *L)
{
  // returns l_hand frame relative to chest frame
  Frame p_hand;
  JntArray q_arm = lua_checkJntArray(L, 1);
  assert(!Mechanics.l_arm_fk_pos_solver->JntToCart(q_arm, p_hand));
  lua_pushFrame(L, p_hand);
  return 1;
}

static int lua_forward_pos_r_arm(lua_State *L)
{
  // returns r_hand frame relative to chest frame
  Frame p_hand;
  JntArray q_arm = lua_checkJntArray(L, 1);
  assert(!Mechanics.r_arm_fk_pos_solver->JntToCart(q_arm, p_hand));
  lua_pushFrame(L, p_hand);
  return 1;
}

static int lua_forward_pos_waist(lua_State *L)
{
  // returns chest frame relative to torso frame
  Frame p_chest;
  JntArray q_waist = lua_checkJntArray(L, 1);
  assert(!Mechanics.waist_fk_pos_solver->JntToCart(q_waist, p_chest));
  lua_pushFrame(L, p_chest);
  return 1;
}

static int lua_forward_pos_l_leg(lua_State *L)
{
  // returns l_foot frame relative to torso frame
  Frame p_foot;
  JntArray q_leg = lua_checkJntArray(L, 1);
  assert(!Mechanics.l_leg_fk_pos_solver->JntToCart(q_leg, p_foot));
  lua_pushFrame(L, p_foot);
  return 1;
}

static int lua_forward_pos_r_leg(lua_State *L)
{
  // returns r_foot frame relative to torso frame
  Frame p_foot;
  JntArray q_leg = lua_checkJntArray(L, 1);
  assert(!Mechanics.r_leg_fk_pos_solver->JntToCart(q_leg, p_foot));
  lua_pushFrame(L, p_foot);
  return 1;
}

static int lua_forward_pos_arms(lua_State *L)
{
  // returns l_hand and r_hand frames relative to chest frame
  JntArray q_l_arm = JntArray(6);
  JntArray q_r_arm = JntArray(6);
  JntArray q_arms = lua_checkJntArray(L, 1);
  for (int i = 0; i < 6; i++)
  {
    q_l_arm(i) = q_arms(i);
    q_r_arm(i) = q_arms(i+6);
  }

  Frame p_l_hand, p_r_hand;
  assert(!Mechanics.l_arm_fk_pos_solver->JntToCart(q_l_arm, p_l_hand));
  assert(!Mechanics.r_arm_fk_pos_solver->JntToCart(q_r_arm, p_r_hand));

  if (lua_istable(L, 2))
  {
    Frame p_chest_offset = lua_checkFrame(L, 2);
    p_l_hand = p_chest_offset*p_l_hand;
    p_r_hand = p_chest_offset*p_r_hand;
  }
  lua_pushFrame(L, p_l_hand);
  lua_pushFrame(L, p_r_hand);
  return 2;
}

static int lua_forward_pos_legs(lua_State *L)
{
  // returns l_foot and r_foot frames relative to torso frame
  JntArray q_l_leg = JntArray(6);
  JntArray q_r_leg = JntArray(6);
  JntArray q_legs = lua_checkJntArray(L, 1);
  for (int i = 0; i < 6; i++)
  {
    q_l_leg(i) = q_legs(i);
    q_r_leg(i) = q_legs(i+6);
  }

  Frame p_l_foot, p_r_foot;
  assert(!Mechanics.l_leg_fk_pos_solver->JntToCart(q_l_leg, p_l_foot));
  assert(!Mechanics.r_leg_fk_pos_solver->JntToCart(q_r_leg, p_r_foot));

  if (lua_istable(L, 2))
  {
    Frame p_torso_offset = lua_checkFrame(L, 2);
    p_l_foot = p_torso_offset*p_l_foot;
    p_r_foot = p_torso_offset*p_r_foot;
  }
  lua_pushFrame(L, p_l_foot);
  lua_pushFrame(L, p_r_foot);
  return 2;
}

// forward velocity kinematics
///////////////////////////////////////////////////////////////////////////

static int lua_forward_vel_head(lua_State *L)
{
  // returns head frame and velocity relative to chest frame
  FrameVel pv_head;
  JntArrayVel qv_head(lua_checkJntArray(L, 1), lua_checkJntArray(L, 2));
  assert(!Mechanics.head_fk_vel_solver->JntToCart(qv_head, pv_head));
  lua_pushFrame(L, pv_head.GetFrame());
  lua_pushTwist(L, pv_head.GetTwist());
  return 2;
}

static int lua_forward_vel_l_arm(lua_State *L)
{
  // returns l_hand frame and velocity relative to chest frame
  FrameVel pv_hand;
  JntArrayVel qv_arm(lua_checkJntArray(L, 1), lua_checkJntArray(L, 2));
  assert(!Mechanics.l_arm_fk_vel_solver->JntToCart(qv_arm, pv_hand));
  lua_pushFrame(L, pv_hand.GetFrame());
  lua_pushTwist(L, pv_hand.GetTwist());
  return 2;
}

static int lua_forward_vel_r_arm(lua_State *L)
{
  // returns r_hand frame and velocity relative to chest frame
  FrameVel pv_hand;
  JntArrayVel qv_arm(lua_checkJntArray(L, 1), lua_checkJntArray(L, 2));
  assert(!Mechanics.r_arm_fk_vel_solver->JntToCart(qv_arm, pv_hand));
  lua_pushFrame(L, pv_hand.GetFrame());
  lua_pushTwist(L, pv_hand.GetTwist());
  return 2;
}

static int lua_forward_vel_waist(lua_State *L)
{
  // returns chest frame and velocity relative to torso frame

  FrameVel pv_chest;
  JntArrayVel qv_waist(lua_checkJntArray(L, 1), lua_checkJntArray(L, 2));
  assert(!Mechanics.waist_fk_vel_solver->JntToCart(qv_waist, pv_chest));
  lua_pushFrame(L, pv_chest.GetFrame());
  lua_pushTwist(L, pv_chest.GetTwist());
  return 2;
}

static int lua_forward_vel_l_leg(lua_State *L)
{
  // returns l_foot frame and velocity relative to torso frame
  FrameVel pv_foot;
  JntArrayVel qv_leg(lua_checkJntArray(L, 1), lua_checkJntArray(L, 2));
  assert(!Mechanics.l_leg_fk_vel_solver->JntToCart(qv_leg, pv_foot));
  lua_pushFrame(L, pv_foot.GetFrame());
  lua_pushTwist(L, pv_foot.GetTwist());
  return 2;
}

static int lua_forward_vel_r_leg(lua_State *L)
{
  // returns r_foot frame and velocity relative to torso frame
  FrameVel pv_foot;
  JntArrayVel qv_leg(lua_checkJntArray(L, 1), lua_checkJntArray(L, 2));
  assert(!Mechanics.r_leg_fk_vel_solver->JntToCart(qv_leg, pv_foot));
  lua_pushFrame(L, pv_foot.GetFrame());
  lua_pushTwist(L, pv_foot.GetTwist());
  return 2;
}

static int lua_forward_vel_arms(lua_State *L)
{
  // returns r_arm and l_arm frames and velocities relative to chest frame
  JntArrayVel qv_arms(lua_checkJntArray(L, 1), lua_checkJntArray(L, 2));
  JntArrayVel qv_l_arm(6);
  JntArrayVel qv_r_arm(6);
  for (int i = 0; i < 6; i++)
  {
    qv_l_arm.q(i) = qv_arms.q(i);
    qv_r_arm.q(i) = qv_arms.q(i+6); 
    qv_l_arm.qdot(i) = qv_arms.qdot(i);
    qv_r_arm.qdot(i) = qv_arms.qdot(i+6); 
  }

  FrameVel pv_l_hand;
  FrameVel pv_r_hand;
  assert(!Mechanics.l_arm_fk_vel_solver->JntToCart(qv_l_arm, pv_l_hand));
  assert(!Mechanics.r_arm_fk_vel_solver->JntToCart(qv_r_arm, pv_r_hand));

  // optional chest frame / velocity offset
  if (lua_istable(L, 3))
  {
    Twist v_chest_offset;
    Frame p_chest_offset = lua_checkFrame(L, 3);
    if (lua_istable(L, 4))
      v_chest_offset = lua_checkTwist(L, 4);

    FrameVel pv_chest_offset = FrameVel(p_chest_offset, v_chest_offset);
    pv_l_hand = pv_chest_offset*pv_l_hand;
    pv_r_hand = pv_chest_offset*pv_r_hand;
  }

  lua_pushFrame(L, pv_l_hand.GetFrame());
  lua_pushFrame(L, pv_r_hand.GetFrame());
  lua_pushTwist(L, pv_l_hand.GetTwist());
  lua_pushTwist(L, pv_r_hand.GetTwist());
  return 4;
}

static int lua_forward_vel_legs(lua_State *L)
{
  // returns r_foot and l_foot frames and velocities relative to torso frame
  JntArrayVel qv_legs(lua_checkJntArray(L, 1), lua_checkJntArray(L, 2));
  JntArrayVel qv_l_leg(6);
  JntArrayVel qv_r_leg(6);
  for (int i = 0; i < 6; i++)
  {
    qv_l_leg.q(i) = qv_legs.q(i);
    qv_r_leg.q(i) = qv_legs.q(i+6); 
    qv_l_leg.qdot(i) = qv_legs.qdot(i);
    qv_r_leg.qdot(i) = qv_legs.qdot(i+6); 
  }

  FrameVel pv_l_foot;
  FrameVel pv_r_foot;
  assert(!Mechanics.l_leg_fk_vel_solver->JntToCart(qv_l_leg, pv_l_foot));
  assert(!Mechanics.r_leg_fk_vel_solver->JntToCart(qv_r_leg, pv_r_foot));

  if (lua_istable(L, 3))
  {
    Twist v_torso_offset;
    Frame p_torso_offset = lua_checkFrame(L, 3);
    if (lua_istable(L, 4))
      v_torso_offset = lua_checkTwist(L, 4);

    FrameVel pv_torso_offset = FrameVel(p_torso_offset, v_torso_offset);
    pv_l_foot = pv_torso_offset*pv_l_foot;
    pv_r_foot = pv_torso_offset*pv_r_foot;
  }

  lua_pushFrame(L, pv_l_foot.GetFrame());
  lua_pushFrame(L, pv_r_foot.GetFrame());
  lua_pushTwist(L, pv_l_foot.GetTwist());
  lua_pushTwist(L, pv_r_foot.GetTwist());
  return 4;
}

// inverse position kinematics
///////////////////////////////////////////////////////////////////////////

static int lua_inverse_pos_head(lua_State *L)
{
  // returns joint positions for head
  Vector p_gaze = lua_checkVector(L, 1);
  JntArray q_head(2);
  head_ik_pos_solver(p_gaze, q_head);
  lua_pushJntArray(L, q_head);
  return 1;
}

static int lua_inverse_pos_l_arm(lua_State *L)
{
  // returns joint positions for l_arm
  Frame p_hand = lua_checkFrame(L, 1);
  JntArray q_arm(6);
  arm_ik_pos_solver(Mechanics.l_arm, p_hand, q_arm);
  lua_pushJntArray(L, q_arm);
  return 1;
}

static int lua_inverse_pos_r_arm(lua_State *L)
{
  // returns joint positions for r_arm
  Frame p_hand = lua_checkFrame(L, 1);
  JntArray q_arm(6);
  arm_ik_pos_solver(Mechanics.r_arm, p_hand, q_arm);
  lua_pushJntArray(L, q_arm);
  return 1;
}

static int lua_inverse_pos_l_leg(lua_State *L)
{
  // returns joint positions for l_leg
  Frame p_foot = lua_checkFrame(L, 1);
  JntArray q_leg(6);
  leg_ik_pos_solver(Mechanics.l_leg, p_foot, q_leg);
  lua_pushJntArray(L, q_leg);
  return 1;
}

static int lua_inverse_pos_r_leg(lua_State *L)
{
  // returns joint positions for r_leg
  Frame p_foot = lua_checkFrame(L, 1);
  JntArray q_leg(6);
  leg_ik_pos_solver(Mechanics.r_leg, p_foot, q_leg);
  lua_pushJntArray(L, q_leg);
  return 1;
}

static int lua_inverse_pos_arms(lua_State *L)
{
  // returns joint positions for both arms
  Frame p_l_hand = lua_checkFrame(L, 1);
  Frame p_r_hand = lua_checkFrame(L, 2);
  if (lua_istable(L, 3))
  {
    Frame p_chest_offset_inv = lua_checkFrame(L, 3).Inverse();
    p_l_hand = p_chest_offset_inv*p_l_hand;
    p_r_hand = p_chest_offset_inv*p_r_hand;
  }

  JntArray q_l_arm(6);
  JntArray q_r_arm(6);
  arm_ik_pos_solver(Mechanics.l_arm, p_l_hand, q_l_arm);
  arm_ik_pos_solver(Mechanics.r_arm, p_r_hand, q_r_arm);

  JntArray q_arms(12);
  for (int i = 0; i < 6; i++)
  {
    q_arms(i) = q_l_arm(i);
    q_arms(i+6) = q_r_arm(i);
  }
  lua_pushJntArray(L, q_arms);
  return 1;
}

static int lua_inverse_pos_legs(lua_State *L)
{
  // returns joint positions for both legs
  Frame p_l_foot = lua_checkFrame(L, 1);
  Frame p_r_foot = lua_checkFrame(L, 2);
  if (lua_istable(L, 3))
  {
    Frame p_torso_offset_inv = lua_checkFrame(L, 3).Inverse();
    p_l_foot = p_torso_offset_inv*p_l_foot;
    p_r_foot = p_torso_offset_inv*p_r_foot;
  }

  JntArray q_l_leg(6);
  JntArray q_r_leg(6);
  leg_ik_pos_solver(Mechanics.l_leg, p_l_foot, q_l_leg);
  leg_ik_pos_solver(Mechanics.r_leg, p_r_foot, q_r_leg);

  JntArray q_legs(12);
  for (int i = 0; i < 6; i++)
  {
    q_legs(i) = q_l_leg(i);
    q_legs(i+6) = q_r_leg(i);
  }
  lua_pushJntArray(L, q_legs);
  return 1;
}

// inverse velocity kinematics
///////////////////////////////////////////////////////////////////////////

static int lua_inverse_vel_l_arm(lua_State *L)
{
  // returns joint positions and velocities for l_arm

  Frame p_hand = lua_checkFrame(L, 1);
  Twist v_hand = lua_checkTwist(L, 2);

  JntArray q_arm(6);
  JntArray qdot_arm(6);
  arm_ik_pos_solver(Mechanics.l_arm, p_hand, q_arm);
  assert(!Mechanics.l_arm_ik_vel_solver->CartToJnt(q_arm, v_hand, qdot_arm));
  lua_pushJntArray(L, q_arm);
  lua_pushJntArray(L, qdot_arm);
  return 2;
}

static int lua_inverse_vel_r_arm(lua_State *L)
{
  // returns joint positions and velocities for r_arm

  Frame p_hand = lua_checkFrame(L, 1);
  Twist v_hand = lua_checkTwist(L, 2);

  JntArray q_arm(6);
  JntArray qdot_arm(6);
  arm_ik_pos_solver(Mechanics.r_arm, p_hand, q_arm);
  assert(!Mechanics.r_arm_ik_vel_solver->CartToJnt(q_arm, v_hand, qdot_arm));

  lua_pushJntArray(L, q_arm);
  lua_pushJntArray(L, qdot_arm);
  return 2;
}

static int lua_inverse_vel_l_leg(lua_State *L)
{
  // returns joint positions and velocities for l_leg
  Frame p_foot = lua_checkFrame(L, 1);
  Twist v_foot = lua_checkTwist(L, 2);

  JntArray q_leg(6);
  JntArray qdot_leg(6);
  leg_ik_pos_solver(Mechanics.l_leg, p_foot, q_leg);
  assert(!Mechanics.l_leg_ik_vel_solver->CartToJnt(q_leg, v_foot, qdot_leg));

  lua_pushJntArray(L, q_leg);
  lua_pushJntArray(L, qdot_leg);
  return 2;
}

static int lua_inverse_vel_r_leg(lua_State *L)
{
  // returns joint positions and velocities for r_leg
  Frame p_foot = lua_checkFrame(L, 1);
  Twist v_foot = lua_checkTwist(L, 2);

  JntArray q_leg(6);
  JntArray qdot_leg(6);
  leg_ik_pos_solver(Mechanics.r_leg, p_foot, q_leg);
  assert(!Mechanics.r_leg_ik_vel_solver->CartToJnt(q_leg, v_foot, qdot_leg));

  lua_pushJntArray(L, q_leg);
  lua_pushJntArray(L, qdot_leg);
  return 2;
}

static int lua_inverse_vel_arms(lua_State *L)
{
  // returns joint positions and velocities for both arms
  Frame p_l_hand = lua_checkFrame(L, 1);
  Frame p_r_hand = lua_checkFrame(L, 2);
  Twist v_l_hand = lua_checkTwist(L, 3);
  Twist v_r_hand = lua_checkTwist(L, 4);

  if (lua_istable(L, 5))
  {
    Twist v_chest_offset;
    Frame p_chest_offset = lua_checkFrame(L, 5);
    if (lua_istable(L, 6))
      v_chest_offset = lua_checkTwist(L, 6);

    FrameVel pv_chest_offset_inv = FrameVel(p_chest_offset, v_chest_offset).Inverse();
    FrameVel pv_l_hand = pv_chest_offset_inv*FrameVel(p_l_hand, v_l_hand);
    FrameVel pv_r_hand = pv_chest_offset_inv*FrameVel(p_r_hand, v_r_hand);
    
    p_l_hand = pv_l_hand.GetFrame();
    p_r_hand = pv_r_hand.GetFrame();
    v_l_hand = pv_l_hand.GetTwist();
    v_r_hand = pv_r_hand.GetTwist();
  }

  JntArray q_l_arm(6);
  JntArray q_r_arm(6);
  JntArray qdot_l_arm(6);
  JntArray qdot_r_arm(6);
  arm_ik_pos_solver(Mechanics.l_arm, p_l_hand, q_l_arm);
  arm_ik_pos_solver(Mechanics.r_arm, p_r_hand, q_r_arm);
  assert(!Mechanics.l_arm_ik_vel_solver->CartToJnt(q_l_arm, v_l_hand, qdot_l_arm));
  assert(!Mechanics.r_arm_ik_vel_solver->CartToJnt(q_r_arm, v_r_hand, qdot_r_arm));

  JntArray q_arms(12);
  JntArray qdot_arms(12);
  for (int i = 0; i < 6; i++)
  {
    q_arms(i) = q_l_arm(i);
    q_arms(i+6) = q_r_arm(i);
    qdot_arms(i) = qdot_l_arm(i);
    qdot_arms(i+6) = qdot_r_arm(i);
  }

  lua_pushJntArray(L, q_arms);
  lua_pushJntArray(L, qdot_arms);
  return 2;
}

static int lua_inverse_vel_legs(lua_State *L)
{
  // returns joint positions and velocities for both legs
  Frame p_l_foot = lua_checkFrame(L, 1);
  Frame p_r_foot = lua_checkFrame(L, 2);
  Twist v_l_foot = lua_checkTwist(L, 3);
  Twist v_r_foot = lua_checkTwist(L, 4);

  if (lua_istable(L, 5))
  {
    Twist v_torso_offset;
    Frame p_torso_offset = lua_checkFrame(L, 5);
    if (lua_istable(L, 6))
      v_torso_offset = lua_checkTwist(L, 6);

    FrameVel pv_torso_offset_inv = FrameVel(p_torso_offset, v_torso_offset).Inverse();
    FrameVel pv_l_foot = pv_torso_offset_inv*FrameVel(p_l_foot, v_l_foot);
    FrameVel pv_r_foot = pv_torso_offset_inv*FrameVel(p_r_foot, v_r_foot);
    
    p_l_foot = pv_l_foot.GetFrame();
    p_r_foot = pv_r_foot.GetFrame();
    v_l_foot = pv_l_foot.GetTwist();
    v_r_foot = pv_r_foot.GetTwist();
  }

  JntArray q_l_leg(6);
  JntArray q_r_leg(6);
  JntArray qdot_l_leg(6);
  JntArray qdot_r_leg(6);
  leg_ik_pos_solver(Mechanics.l_leg, p_l_foot, q_l_leg);
  leg_ik_pos_solver(Mechanics.r_leg, p_r_foot, q_r_leg);
  assert(!Mechanics.l_leg_ik_vel_solver->CartToJnt(q_l_leg, v_l_foot, qdot_l_leg));
  assert(!Mechanics.r_leg_ik_vel_solver->CartToJnt(q_r_leg, v_r_foot, qdot_r_leg));

  JntArray q_legs(12);
  JntArray qdot_legs(12);
  for (int i = 0; i < 6; i++)
  {
    q_legs(i) = q_l_leg(i);
    q_legs(i+6) = q_r_leg(i);
    qdot_legs(i) = qdot_l_leg(i);
    qdot_legs(i+6) = qdot_r_leg(i);
  }

  lua_pushJntArray(L, q_legs);
  lua_pushJntArray(L, qdot_legs);
  return 2;
}

// utilities
///////////////////////////////////////////////////////////////////////////

static int lua_get_head_jacobian(lua_State *L)
{
  // returns head jacobian matrix relative to chest frame
  Jacobian J_head;
  JntArray q_head = lua_checkJntArray(L, 1);
  assert(!Mechanics.head_jnt_to_jac_solver->JntToJac(q_head, J_head));
  lua_pushJacobian(L, J_head);
}

static int lua_get_l_arm_jacobian(lua_State *L)
{
  // returns l_arm jacobian matrix relative to chest frame
  Jacobian J_l_arm;
  JntArray q_l_arm = lua_checkJntArray(L, 1);
  assert(!Mechanics.l_arm_jnt_to_jac_solver->JntToJac(q_l_arm, J_l_arm));
  lua_pushJacobian(L, J_l_arm);
}

static int lua_get_r_arm_jacobian(lua_State *L)
{
  // returns r_arm jacobian matrix relative to chest frame
  Jacobian J_r_arm;
  JntArray q_r_arm = lua_checkJntArray(L, 1);
  assert(!Mechanics.r_arm_jnt_to_jac_solver->JntToJac(q_r_arm, J_r_arm));
  lua_pushJacobian(L, J_r_arm);
}

static int lua_get_waist_jacobian(lua_State *L)
{
  // returns waist jacobian matrix relative to torso frame
  Jacobian J_waist;
  JntArray q_waist = lua_checkJntArray(L, 1);
  assert(!Mechanics.waist_jnt_to_jac_solver->JntToJac(q_waist, J_waist));
  lua_pushJacobian(L, J_waist);
}

static int lua_get_l_leg_jacobian(lua_State *L)
{
  // returns l_leg jacobian matrix relative to torso frame
  Jacobian J_l_leg;
  JntArray q_l_leg = lua_checkJntArray(L, 1);
  assert(!Mechanics.l_leg_jnt_to_jac_solver->JntToJac(q_l_leg, J_l_leg));
  lua_pushJacobian(L, J_l_leg);
}

static int lua_get_r_leg_jacobian(lua_State *L)
{
  // returns r_leg jacobian matrix relative to torso frame
  Jacobian J_r_leg;
  JntArray q_r_leg = lua_checkJntArray(L, 1);
  assert(!Mechanics.r_leg_jnt_to_jac_solver->JntToJac(q_r_leg, J_r_leg));
  lua_pushJacobian(L, J_r_leg);
}

static const struct luaL_reg Kinematics_lib [] = {
  {"forward_pos_head", lua_forward_pos_head},
  {"forward_pos_l_arm", lua_forward_pos_l_arm},
  {"forward_pos_r_arm", lua_forward_pos_r_arm},
  {"forward_pos_waist", lua_forward_pos_waist},
  {"forward_pos_l_leg", lua_forward_pos_l_leg},
  {"forward_pos_r_leg", lua_forward_pos_r_leg},
  {"forward_pos_arms", lua_forward_pos_arms},
  {"forward_pos_legs", lua_forward_pos_legs},
  {"forward_vel_head", lua_forward_vel_head},
  {"forward_vel_l_arm", lua_forward_vel_l_arm},
  {"forward_vel_r_arm", lua_forward_vel_r_arm},
  {"forward_vel_waist", lua_forward_vel_waist},
  {"forward_vel_l_leg", lua_forward_vel_l_leg},
  {"forward_vel_r_leg", lua_forward_vel_r_leg},
  {"forward_vel_arms", lua_forward_vel_arms},
  {"forward_vel_legs", lua_forward_vel_legs},
  {"inverse_pos_head", lua_inverse_pos_head},
  {"inverse_pos_l_arm", lua_inverse_pos_l_arm},
  {"inverse_pos_r_arm", lua_inverse_pos_r_arm},
  {"inverse_pos_l_leg", lua_inverse_pos_l_leg},
  {"inverse_pos_r_leg", lua_inverse_pos_r_leg},
  {"inverse_pos_arms", lua_inverse_pos_arms},
  {"inverse_pos_legs", lua_inverse_pos_legs},
  {"inverse_vel_l_arm", lua_inverse_vel_l_arm},
  {"inverse_vel_r_arm", lua_inverse_vel_r_arm},
  {"inverse_vel_l_leg", lua_inverse_vel_l_leg},
  {"inverse_vel_r_leg", lua_inverse_vel_r_leg},
  {"inverse_vel_arms", lua_inverse_vel_arms},
  {"inverse_vel_legs", lua_inverse_vel_legs},
  {"get_head_jacobian", lua_get_head_jacobian},
  {"get_l_arm_jacobian", lua_get_l_arm_jacobian},
  {"get_r_arm_jacobian", lua_get_r_arm_jacobian},
  {"get_waist_jacobian", lua_get_waist_jacobian},
  {"get_l_leg_jacobian", lua_get_l_leg_jacobian},
  {"get_r_leg_jacobian", lua_get_r_leg_jacobian},
  {NULL, NULL}
};

extern "C"
int luaopen_Kinematics(lua_State *L)
{
  luaL_register(L, "Kinematics", Kinematics_lib);
  return 1;
}
