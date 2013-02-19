#include "Mechanics.h"

_Mechanics Mechanics;

_Mechanics::_Mechanics()
{

  gravity = Vector(0, 0, -9.81);

  // coordinate transforms
  /////////////////////////////////////////////////////////////////////////

  // head
  clavicle_transform = Frame(Vector(0, 0, 0));
  neck_transform = Frame(Vector(0, 0, 0));
  head_transform = Frame(Vector(0, 0, 0));

  // l_arm
  l_shoulder_transform = Frame(Vector(0, 0.219, 0));
  l_arm_transform = Frame(Vector(0.03, 0, -0.246));
  l_forearm_transform = Frame(Vector(-0.03, 0, -0.242));
  l_hand_transform = Frame(Vector(0, 0, 0));

  // r_arm 
  r_shoulder_transform = Frame(Vector(0, -0.219, 0));
  r_arm_transform = Frame(Vector(0.03, 0, -0.246));
  r_forearm_transform = Frame(Vector(-0.03, 0, -0.242));
  r_hand_transform = Frame(Vector(0, 0, 0));

  // waist
  torso_waist_transform = Frame(Vector(-0.001, 0, 0.372));
  waist_chest_transform = Frame(Vector(0, 0, 0.144));

  // l_leg
  l_hip_transform = Frame(Vector(0, 0.097, 0));
  l_thigh_transform = Frame(Vector(0.00255, 0, -0.379025));
  l_shin_transform = Frame(Vector(0, 0, -0.3800));
  l_foot_transform = Frame(Vector(0, 0, -0.0487));

  // r_leg
  r_hip_transform = Frame(Vector(0, -0.097, 0));
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
    8.282695889,
    Vector(0.008418388, -0.000536018, -0.110449252),
    RotationalInertia(0.089047014, 0.091979393, 0.010624728,
                      -0.000095894, -0.009268308, 0.000602044)
  );
  l_forearm_inertia = RigidBodyInertia(
    3.134156798,
    Vector(-0.030261110, 0.000635042, -0.144485627),
    RotationalInertia(0.016765270, 0.016175558, 0.002923078,
                      0.000029334, 0.000471794, 0.000258425)
  );
  l_hand_inertia = RigidBodyInertia(
    2.130412358,
    Vector(0.002252340, -0.007063941, -0.110456457),
    RotationalInertia(0.005981976, 0.005688715, 0.002226253,
                      0.000084308, -0.000065941, 0.000777435)
  );

  // r_arm
  r_arm_inertia = RigidBodyInertia(
    8.282695889,
    Vector(0.008418388, -0.000536018, -0.110449252),
    RotationalInertia(0.089047014, 0.091979393, 0.010624728,
                      -0.000095894, -0.009268308, 0.000602044)
  );
  r_forearm_inertia = RigidBodyInertia(
    3.134156798,
    Vector(-0.030261110, 0.000635042, -0.144485627),
    RotationalInertia(0.016765270, 0.016175558, 0.002923078,
                      0.000029334, 0.000471794, 0.000258425)
  );
  r_hand_inertia = RigidBodyInertia(
    2.130412358,
    Vector(-0.000841234, 0.006961820, -0.110456457),
    RotationalInertia(0.005981410, 0.005691425, 0.002228396,
                      0.000078228, 0.000157972, -0.000774389)
  );

  // waist
  chest_inertia = RigidBodyInertia(
    3.339282487,
    Vector(-0.036092089, -0.002796037, 0.100620268),
    RotationalInertia(0.048654895, 0.052096069, 0.016272686,
                      0.000616885, -0.013012592, -0.001364607)
  );

  // l_leg
  l_thigh_inertia = RigidBodyInertia(
    2.214889327,
    Vector(-0.009015634, 0.016546555, -0.159241491),
    RotationalInertia(0.021142224, 0.024067859, 0.008757645,
                      -0.002091864, -0.001007431, -0.001750657)
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

  // base
  torso_inertia = RigidBodyInertia(
    9.846994362,
    Vector(0.009821043, 0.000461078, 0.205279880),
    RotationalInertia(0.161838147, 0.120693934, 0.120189496,
                      0.000446170, -0.000151051, 0.000329275)
  );

  // kinematic chains
  /////////////////////////////////////////////////////////////////////////

  // head
  head.addSegment(Segment("clavicle", Joint(Joint::None),
    clavicle_transform));
  head.addSegment(Segment("head_yaw", Joint(Joint::RotZ),
    neck_transform, neck_inertia));
  head.addSegment(Segment("head_pitch", Joint(Joint::RotY), 
    head_transform, head_inertia));
  head.addSegment(Segment("head"));

  // l_arm
  l_arm.addSegment(Segment("l_shoulder", Joint(Joint::None),
    l_shoulder_transform));
  l_arm.addSegment(Segment("l_shoulder_pitch", Joint(Joint::RotY)));
  l_arm.addSegment(Segment("l_shoulder_roll", Joint(Joint::RotX)));
  l_arm.addSegment(Segment("l_shoulder_yaw", Joint(Joint::RotZ), 
    l_arm_transform, l_arm_inertia)); 
  l_arm.addSegment(Segment("l_elbow_pitch", Joint(Joint::RotY),
    l_forearm_transform, l_forearm_inertia));
  l_arm.addSegment(Segment("l_wrist_yaw", Joint(Joint::RotZ)));
  l_arm.addSegment(Segment("l_wrist_roll", Joint(Joint::RotY),
    l_hand_transform, l_hand_inertia));
  waist.addSegment(Segment("l_hand"));

  // r_arm
  r_arm.addSegment(Segment("r_shoulder", Joint(Joint::None),
    r_shoulder_transform));
  r_arm.addSegment(Segment("r_shoulder_pitch", Joint(Joint::RotY)));
  r_arm.addSegment(Segment("r_shoulder_roll", Joint(Joint::RotX)));
  r_arm.addSegment(Segment("r_shoulder_yaw", Joint(Joint::RotZ), 
    r_arm_transform, r_arm_inertia)); 
  r_arm.addSegment(Segment("r_elbow_pitch", Joint(Joint::RotY),
    r_forearm_transform, r_forearm_inertia));
  r_arm.addSegment(Segment("r_wrist_yaw", Joint(Joint::RotZ)));
  r_arm.addSegment(Segment("r_wrist_roll", Joint(Joint::RotY),
    r_hand_transform, r_hand_inertia));
  waist.addSegment(Segment("r_hand"));

  // waist
  waist.addSegment(Segment("waist", Joint(Joint::None),
    torso_waist_transform, torso_inertia));
  waist.addSegment(Segment("waist_yaw", Joint(Joint::RotZ),
    waist_chest_transform, chest_inertia));
  waist.addSegment(Segment("chest"));

  // l_leg
  l_leg.addSegment(Segment("l_hip", Joint(Joint::None),
    l_hip_transform));
  l_leg.addSegment(Segment("l_hip_yaw", Joint(Joint::RotZ)));
  l_leg.addSegment(Segment("l_hip_roll", Joint(Joint::RotX)));
  l_leg.addSegment(Segment("l_hip_pitch", Joint(Joint::RotY), 
    l_thigh_transform, l_thigh_inertia)); 
  l_leg.addSegment(Segment("l_knee_pitch", Joint(Joint::RotY),
    l_shin_transform, l_shin_inertia));
  l_leg.addSegment(Segment("l_ankle_pitch", Joint(Joint::RotY)));
  l_leg.addSegment(Segment("l_ankle_roll", Joint(Joint::RotX),
    l_foot_transform, l_foot_inertia));
  l_leg.addSegment(Segment("l_foot"));

  // r_leg
  r_leg.addSegment(Segment("r_hip", Joint(Joint::None),
    r_hip_transform));
  r_leg.addSegment(Segment("r_hip_yaw", Joint(Joint::RotZ)));
  r_leg.addSegment(Segment("r_hip_roll", Joint(Joint::RotX)));
  r_leg.addSegment(Segment("r_hip_pitch", Joint(Joint::RotY), 
    r_thigh_transform, r_thigh_inertia)); 
  r_leg.addSegment(Segment("r_knee_pitch", Joint(Joint::RotY),
    r_shin_transform, r_shin_inertia));
  r_leg.addSegment(Segment("r_ankle_pitch", Joint(Joint::RotY)));
  r_leg.addSegment(Segment("r_ankle_roll", Joint(Joint::RotX),
    r_foot_transform, r_foot_inertia));
  r_leg.addSegment(Segment("r_foot"));

  // base
  base.addSegment(Segment("base_x", Joint(Joint::TransX)));
  base.addSegment(Segment("base_y", Joint(Joint::TransY)));
  base.addSegment(Segment("base_z", Joint(Joint::TransZ)));
  base.addSegment(Segment("base_roll", Joint(Joint::RotX)));
  base.addSegment(Segment("base_pitch", Joint(Joint::RotY)));
  base.addSegment(Segment("base_yaw", Joint(Joint::RotZ)));
  base.addSegment(Segment("torso"));

  // kinematic trees
  /////////////////////////////////////////////////////////////////////////
  body.addChain(base, "root");
  body.addChain(l_leg, "torso");
  body.addChain(r_leg, "torso");
  body.addChain(waist, "torso");
  body.addChain(l_arm, "chest");
  body.addChain(r_arm, "chest");
  body.addChain(head, "chest");

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

  // body
  body_id_solver = new TreeIdSolver_RNE(body, gravity);
};
