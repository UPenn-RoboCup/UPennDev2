assert(Config, 'Need a pre-existing Config table!')
local vector = require'vector'
local carray = require'carray'

------------------------------------
-- For the arm FSM
local arm = {}

arm.default_hand_mass = 0.4

--Gripper end position offsets (Y is inside)
----------------------------------------
-- Default UCLA gripper: {0.15, 0, 0}
-- Chopsticks: {0.35, 0, 0}

arm.handoffset={}
local offset_ucla_hand = {0.15,0,0}
local offset_chipsticks = {0.30,0,0}
local offset_wrist = {0,0,0}
arm.handoffset.left = offset_wrist
arm.handoffset.right = offset_wrist

--arm.handoffset.left=offset_ucla_hand
--arm.handoffset.right =offset_ucla_hand

--Torques for finger controls
arm.torque={}
arm.torque.movement = 5
arm.torque.open = -10
arm.torque.grip_hose = 10
arm.torque.grip_drill = 10
arm.torque.grip_drill_trigger1 = 40
arm.torque.grip_drill_trigger2 = 40


--Old Arm planner variables
arm.plan={}
arm.plan.max_margin = math.pi/6
arm.plan.dt_step0 = 0.1
arm.plan.dt_step = 0.2
arm.plan.dt_step_min = 0.02
arm.plan.dt_step_min = 0.2 --no speedup
arm.plan.search_step = 0.25


--for SJ's jacobian stuff------------------------------
arm.plan.dt_step0_jacobian = 0.05
arm.plan.dt_step_jacobian = 0.1
arm.plan.dt_step_min_jacobian = 0.02
arm.plan.scale_limit={0.05,2}

arm.plan.dt_step0_jacobian = 0.1
arm.plan.dt_step_jacobian = 0.2


-- Arm speed limits
arm.torso_comp_limit = vector.new({0.02,0.01})

--testing for speeding up
arm.overspeed_factor = 3

--Servo limit
-- 30rpm 30rpm 30rpm 30rpm 20rpm 20rpm 20rpm
-- = 180 180 180 180 120 120 120 degree/sec
arm.vel_angular_limit = vector.new({20,20,20,20,30,30,30})*DEG_TO_RAD
arm.vel_angular_limit_init = vector.new({20,20,20,20,30,30,30})*DEG_TO_RAD
arm.vel_linear_limit = vector.new({0.02,0.02,0.02, 30*DEG_TO_RAD,30*DEG_TO_RAD,30*DEG_TO_RAD})
arm.vel_waist_limit = vector.new({3,3})*DEG_TO_RAD
arm.shoulder_yaw_limit = 30*DEG_TO_RAD
arm.torso_comp_limit = vector.new({0.06,0.03})
------------------------------------------------------

--Pose 1 left and right arm poses
--SJ: now arm ready poses are hand-invariant (to handle asymmetric hands and stuff)
arm.trLArm0 = {0.0, 0.25,-0.25,0,0,0}
arm.trRArm0 = {0.0, -0.25,-0.25,0,0,0}
arm.ShoulderYaw0=vector.new({0.1,-0.1})*DEG_TO_RAD

--Arm State specific infos
armfsm = {}

armfsm.teleop = {}

--FOR DRC TESTBED
--Table height: 1.04 from grpund, 0.11 from waist center
--Drill handle height: 1.15 from ground, 0.22 from waist center
--Valve center: 1.09 from ground, 0.16 from waist center
--Hose center: 1.25 from groundm 0.32 from waist center


armfsm.teleop.lhand_rpy0={0,0,0}
armfsm.teleop.rhand_rpy0={0,0,0}

--straight arm, works with mk1 arm
armfsm.teleop.arminit={
  {'move0',nil,{0.15,-0.25,-0.15,0,0,0}},
  {'move0',nil,{0.23,-0.25,0.24,0,0,0}},
  {'move0',nil,{0.33,-0.15,0.24,0,0,0}},
}
armfsm.teleop.armuninit={
  {'move0',nil,{0.23,-0.25, 0.24  ,0,0,0}},
  {'move0',nil,{0.15,-0.25,-0.15  ,0,0,0}},
  {'move0',nil,{0.0,-0.25, -0.25,0,0,0}},
}


--straight arm, works with mk2 stock arm
armfsm.teleop.arminit={
  {'move0',nil,{0.15,-0.25,-0.15,0,0,0}},
  {'move0',nil,{0.25,-0.25,0.24,0,0,0}},
  {'move0',nil,{0.40,-0.15,0.24,0,0,0}},
}
armfsm.teleop.armuninit={
  {'move0',nil,{0.25,-0.25, 0.24  ,0,0,0}},
  {'move0',nil,{0.15,-0.25,-0.15  ,0,0,0}},
  {'move0',nil,{0.0,-0.25, -0.25,0,0,0}},   
}



--FOR LONGARM VER.
arm.trLArm0 = {0.15, 0.25,-0.25,0,0,0}
arm.trRArm0 = {0.15, -0.25,-0.25,0,0,0}

--arm.trLArm0 = {0.10, 0.25,-0.45,0,0,0}
--arm.trRArm0 = {0.10, -0.25,-0.45,0,0,0}


armfsm.teleop.arminit={
  {'move0',nil,{0.25,-0.25,-0.15,0,0,0}},
  {'move0',nil,{0.30,-0.25,0.24,0,0,0}},
  {'move0',nil,{0.54,-0.15,0.24,0,0,0}},
}
armfsm.teleop.armuninit={
  {'move0',nil,{0.30,-0.25, 0.24  ,0,0,0}},
  {'move0',nil,{0.25,-0.25,-0.15  ,0,0,0}},
  {'move0',nil,arm.trRArm0},   
}



------------------------------------
-- Associate with the table
Config.arm     = arm
Config.armfsm  = armfsm



return Config
