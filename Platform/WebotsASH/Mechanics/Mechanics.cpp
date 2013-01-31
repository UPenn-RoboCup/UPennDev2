#include "Mechanics.h"

_Mechanics Mechanics;

_Mechanics::_Mechanics()
{

  gravity = Vector(0, 0, -9.81);

  // coordinate transforms
  /////////////////////////////////////////////////////////////////////////

  // head
  chest_transform = Frame(Vector(0, 0, 0));
  neck_transform = Frame(Vector(0, 0, 0));
  head_transform = Frame(Vector(0, 0, 0));

  // l_arm
  l_chest_transform = Frame(Vector(0, 0, 0));
  l_arm_transform = Frame(Vector(0, 0, 0));
  l_forearm_transform = Frame(Vector(0, 0, 0));
  l_hand_transform = Frame(Vector(0, 0, 0));

  // r_arm 
  r_chest_transform = Frame(Vector(0, 0, 0));
  r_arm_transform = Frame(Vector(0, 0, 0));
  r_forearm_transform = Frame(Vector(0, 0, 0));
  r_hand_transform = Frame(Vector(0, 0, 0));

  // waist
  torso_waist_transform = Frame(Vector(0, 0, 0));
  waist_chest_transform = Frame(Vector(0, 0, 0));

  // l_leg
  l_torso_transform = Frame(Vector(0, 0.097, 0));
  l_thigh_transform = Frame(Vector(0.00255, 0, -0.379025));
  l_shin_transform = Frame(Vector(0, 0, -0.3800));
  l_foot_transform = Frame(Vector(0, 0, -0.0487));

  // r_leg
  r_torso_transform = Frame(Vector(0, -0.097, 0));
  r_thigh_transform = Frame(Vector(0.00255, 0, -0.379025));
  r_shin_transform = Frame(Vector(0, 0, -0.3800));
  r_foot_transform = Frame(Vector(0, 0, -0.0487));

  // link inertias
  /////////////////////////////////////////////////////////////////////////

  // head
  neck_inertia = RigidBodyInertia(
    0,
    Vector::Zero(),
    RotationalInertia()
  );
  head_inertia = RigidBodyInertia(
    0,
    Vector::Zero(),
    RotationalInertia()
  );

  // l_arm
  l_arm_inertia = RigidBodyInertia(
    0,
    Vector::Zero(),
    RotationalInertia()
  );
  l_forearm_inertia = RigidBodyInertia(
    0,
    Vector::Zero(),
    RotationalInertia()
  );
  l_hand_inertia = RigidBodyInertia(
    0,
    Vector::Zero(),
    RotationalInertia()
  );

  // r_arm
  r_arm_inertia = RigidBodyInertia(
    0,
    Vector::Zero(),
    RotationalInertia()
  );
  r_forearm_inertia = RigidBodyInertia(
    0,
    Vector::Zero(),
    RotationalInertia()
  );
  r_hand_inertia = RigidBodyInertia(
    0,
    Vector::Zero(),
    RotationalInertia()
  );

  // waist
  torso_inertia = RigidBodyInertia(
    9.846994362,
    Vector(0.008823089, 0.000468598, 0.140308293),
    RotationalInertia(0.161770745, 0.120602231, 0.120207907,
                      0.000443915, -0.000174501, 0.000354588)
  );
  chest_inertia = RigidBodyInertia(
    3.339282487,
    Vector(-0.036092089, -0.002796037, -0.043379732),
    RotationalInertia(0.014820430, 0.013937830, 0.011896701,
                      0.000279902, -0.000885669, -0.000425140)
  );

  // l_leg
  l_thigh_inertia = RigidBodyInertia(
    2.214889327,
    Vector(-0.009015634, -0.016546555, -0.159241491),
    RotationalInertia(0.021142224, 0.024067859, 0.008757645,
                      0.002091864, -0.001007431, 0.001750657)
  );
  l_shin_inertia = RigidBodyInertia(
    2.239063803,
    Vector(-0.025656333, -0.000475219, -0.126284289),
    RotationalInertia(0.021748921, 0.027028523, 0.010583518,
                      0.000056002, 0.004889713, 0.000035634)
  );
  l_foot_inertia = RigidBodyInertia(
    0.589284455,
    Vector(0.003616028,  0.000275946, -0.029612515),
    RotationalInertia(0.000545430, 0.001825442, 0.002054387,
                      0.000004512, -0.000179602, -0.000001187)
  );

  // r_leg
  r_thigh_inertia = RigidBodyInertia(
    2.214889327,
    Vector(-0.009015634, -0.016546555, -0.159241491),
    RotationalInertia(0.021142224, 0.024067859, 0.008757645,
                      0.002091864, -0.001007431, 0.001750657)
  );
  r_shin_inertia = RigidBodyInertia(
    2.239063803,
    Vector(-0.025656333, -0.000475219, -0.126284289),
    RotationalInertia(0.021748921, 0.027028523, 0.010583518,
                      0.000056002, 0.004889713, 0.000035634)
  );
  r_foot_inertia = RigidBodyInertia(
    0.589284455,
    Vector(0.003616028,  0.000275946, -0.029612515),
    RotationalInertia(0.000545430, 0.001825442, 0.002054387,
                      0.000004512, -0.000179602, -0.000001187)
  );

  // kinematic chains
  /////////////////////////////////////////////////////////////////////////

  // head
  head.addSegment(Segment(Joint(Joint::None),
    chest_transform));
  head.addSegment(Segment(Joint(Joint::RotZ),
    neck_transform, neck_inertia));
  head.addSegment(Segment(Joint(Joint::RotY), 
    head_transform, head_inertia)); 

  // l_arm
  l_arm.addSegment(Segment(Joint(Joint::None),
    l_chest_transform));
  l_arm.addSegment(Segment(Joint(Joint::RotY)));
  l_arm.addSegment(Segment(Joint(Joint::RotX)));
  l_arm.addSegment(Segment(Joint(Joint::RotZ), 
    l_arm_transform, l_arm_inertia)); 
  l_arm.addSegment(Segment(Joint(Joint::RotY),
    l_forearm_transform, l_forearm_inertia));
  l_arm.addSegment(Segment(Joint(Joint::RotZ)));
  l_arm.addSegment(Segment(Joint(Joint::RotY),
    l_hand_transform, l_hand_inertia));

  // r_arm
  r_arm.addSegment(Segment(Joint(Joint::None),
    r_chest_transform));
  r_arm.addSegment(Segment(Joint(Joint::RotY)));
  r_arm.addSegment(Segment(Joint(Joint::RotX)));
  r_arm.addSegment(Segment(Joint(Joint::RotZ), 
    r_arm_transform, r_arm_inertia)); 
  r_arm.addSegment(Segment(Joint(Joint::RotY),
    r_forearm_transform, r_forearm_inertia));
  r_arm.addSegment(Segment(Joint(Joint::RotZ)));
  r_arm.addSegment(Segment(Joint(Joint::RotY),
    r_hand_transform, r_hand_inertia));

  // waist
  waist.addSegment(Segment(Joint(Joint::None),
    torso_waist_transform, torso_inertia));
  waist.addSegment(Segment(Joint(Joint::RotZ),
    waist_chest_transform, chest_inertia));

  // l_leg
  l_leg.addSegment(Segment(Joint(Joint::None),
    l_torso_transform));
  l_leg.addSegment(Segment(Joint(Joint::RotZ)));
  l_leg.addSegment(Segment(Joint(Joint::RotX)));
  l_leg.addSegment(Segment(Joint(Joint::RotY), 
    l_thigh_transform, l_thigh_inertia)); 
  l_leg.addSegment(Segment(Joint(Joint::RotY),
    l_shin_transform, l_shin_inertia));
  l_leg.addSegment(Segment(Joint(Joint::RotY)));
  l_leg.addSegment(Segment(Joint(Joint::RotX),
    l_foot_transform, l_foot_inertia));

  // r_leg
  r_leg.addSegment(Segment(Joint(Joint::None),
    r_torso_transform));
  r_leg.addSegment(Segment(Joint(Joint::RotZ)));
  r_leg.addSegment(Segment(Joint(Joint::RotX)));
  r_leg.addSegment(Segment(Joint(Joint::RotY), 
    r_thigh_transform, r_thigh_inertia)); 
  r_leg.addSegment(Segment(Joint(Joint::RotY),
    r_shin_transform, r_shin_inertia));
  r_leg.addSegment(Segment(Joint(Joint::RotY)));
  r_leg.addSegment(Segment(Joint(Joint::RotX),
    r_foot_transform, r_foot_inertia));

  // kinematic and dynamic solvers
  /////////////////////////////////////////////////////////////////////////

  // head
  head_fk_pos_solver = new ChainFkSolverPos_recursive(head);
  head_fk_vel_solver = new ChainFkSolverVel_recursive(head);
  head_ik_vel_solver = new ChainIkSolverVel_pinv(head);
  head_jnt_to_jac_solver = new ChainJntToJacSolver(head);
  head_dynamic_param = new ChainDynParam(head, gravity); 

  // l_arm
  l_arm_fk_pos_solver = new ChainFkSolverPos_recursive(l_arm);
  l_arm_fk_vel_solver = new ChainFkSolverVel_recursive(l_arm);
  l_arm_ik_vel_solver = new ChainIkSolverVel_pinv(l_arm);
  l_arm_jnt_to_jac_solver = new ChainJntToJacSolver(l_arm);
  l_arm_dynamic_param = new ChainDynParam(l_arm, gravity); 

  // r_arm
  r_arm_fk_pos_solver = new ChainFkSolverPos_recursive(r_arm);
  r_arm_fk_vel_solver = new ChainFkSolverVel_recursive(r_arm);
  r_arm_ik_vel_solver = new ChainIkSolverVel_pinv(r_arm);
  r_arm_jnt_to_jac_solver = new ChainJntToJacSolver(r_arm);
  r_arm_dynamic_param = new ChainDynParam(r_arm, gravity); 

  // waist
  waist_fk_pos_solver = new ChainFkSolverPos_recursive(waist);
  waist_fk_vel_solver = new ChainFkSolverVel_recursive(waist);
  waist_ik_vel_solver = new ChainIkSolverVel_pinv(waist);
  waist_jnt_to_jac_solver = new ChainJntToJacSolver(waist);
  waist_dynamic_param = new ChainDynParam(waist, gravity); 
  
  // l_leg
  l_leg_fk_pos_solver = new ChainFkSolverPos_recursive(l_leg);
  l_leg_fk_vel_solver = new ChainFkSolverVel_recursive(l_leg);
  l_leg_ik_vel_solver = new ChainIkSolverVel_pinv(l_leg);
  l_leg_jnt_to_jac_solver = new ChainJntToJacSolver(l_leg);
  l_leg_dynamic_param = new ChainDynParam(l_leg, gravity); 

  // r_leg
  r_leg_fk_pos_solver = new ChainFkSolverPos_recursive(r_leg);
  r_leg_fk_vel_solver = new ChainFkSolverVel_recursive(r_leg);
  r_leg_ik_vel_solver = new ChainIkSolverVel_pinv(r_leg);
  r_leg_jnt_to_jac_solver = new ChainJntToJacSolver(r_leg);
  r_leg_dynamic_param = new ChainDynParam(r_leg, gravity); 
};
