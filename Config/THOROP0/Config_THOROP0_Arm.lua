assert(Config, 'Need a pre-existing Config table!')
local vector = require'vector'
local T = require'Transform'
local tr6D = require'Transform'.transform6D

------------------------------------
-- For the arm FSM
-- Weights: cusage, cdiff, ctight
local arm = {}

-- This comes after the walkInit
arm.init = {}
-- Pitch up
arm.init[1] = {
	left = {
		tr=tr6D{0.2, 0.25, 0.15, 0, -60*DEG_TO_RAD,0},
		timeout=20,
		via='jacobian_preplan', weights = {0,1,0}
	},
	right = {
		tr=tr6D{0.2, -0.25, 0.2, 0, -60*DEG_TO_RAD, 0},
		--q = {0,0,0, 0, 0,0,0},
		timeout=20,
		via='jacobian_preplan',
		weights = {0,1,0},
	}
}

-- Now go to yaw
arm.init[2] = {
	left = {
		tr=tr6D{0.25, 0.25, 0.2,    0, 0, -45*DEG_TO_RAD},
		qArmGuess = vector.new{70,25,-28, -150, 10,-80,-90}*DEG_TO_RAD,
		timeout=20,
		via='jacobian_preplan', weights = {0,0,0,1}
	},
	right = {
		tr=tr6D{0.2, -0.25, 0.2, 0*DEG_TO_RAD, 0*DEG_TO_RAD, 45*DEG_TO_RAD},
		timeout=20,
		via='jacobian_preplan',
		weights = {0,0,0,1},
	}
}

-- From some random manipulation point
arm.ready = {}
arm.ready[1] = {
	left = {
		tr=tr6D{0.35, 0.25, 0.2,    0, 0, -30*DEG_TO_RAD},
		qArmGuess = {0*DEG_TO_RAD,-90*DEG_TO_RAD,0*DEG_TO_RAD, 0, 0,0,0},
		timeout=20,
		via='jacobian_preplan', weights = {0,1,0,1}
	},
	right = {
		tr=tr6D{0.2, -0.25, 0.2, 0*DEG_TO_RAD, 0*DEG_TO_RAD, 45*DEG_TO_RAD},
		timeout=20,
		via='jacobian_preplan',
		weights = {1,0,0,1},
	}
}

--Gripper end position offsets (Y is inside)
arm.handoffset = {}
--0.130 + 0.60+0.50
--arm.handoffset.gripper = {0.241,0,0} --Default gripper
arm.handoffset.gripper = {0.23,0,0} --Default gripper (VT)
--0.130+0.139+0.80-0.10
arm.handoffset.outerhook = {0.339,0,0.060} --Single hook (for door)
--0.130 + 0.140+0.80-0.10
--arm.handoffset.chopstick = {0.340,0,0} --Two rod (for valve)
--FROM EMPIRICAL DATA
arm.handoffset.chopstick = {0.440,0,0} --Two rod (for valve)
--New 3 finger gripper
arm.handoffset.gripper3 = {0.28,-0.05,0}




------------------------------------------------------------------------------
------------------------------------------------------------------------------

arm.torque={}
arm.torque.movement = 5
arm.torque.open = -10
arm.torque.grip_hose = 10
arm.torque.grip_drill = 10
arm.torque.grip_drill_trigger1 = 40
arm.torque.grip_drill_trigger2 = 40

arm.handoffset={}
local offset_ucla_hand = {0.15,0,0}
local offset_chipsticks = {0.30,0,0}
local offset_wrist = {0,0,0}
arm.handoffset.left = offset_wrist
arm.handoffset.right = offset_wrist

--Walk arm pose and shoulder angle
arm.trLArm0 = {0.0, 0.25,-0.25,0,0,0}
arm.trRArm0 = {0.0, -0.25,-0.25,0,0,0}
arm.ShoulderYaw0=vector.new({-1,1})*DEG_TO_RAD

arm.vel_angular_limit = vector.new({20,20,20,20,30,30,30})*DEG_TO_RAD
arm.vel_angular_limit_init = vector.new({20,20,20,20,30,30,30})*DEG_TO_RAD
arm.vel_linear_limit = vector.new({0.02,0.02,0.02, 30*DEG_TO_RAD,30*DEG_TO_RAD,30*DEG_TO_RAD})
arm.vel_waist_limit = vector.new({3,3})*DEG_TO_RAD
arm.shoulder_yaw_limit = 30*DEG_TO_RAD
arm.torso_comp_limit = vector.new({0.06,0.03})

--Old teleop init/uninit sequence
armfsm = {}
armfsm.teleop = {}
armfsm.teleop.arminit={
  {'move0',nil,{0.25,-0.25,-0.15,0,0,0}},
  {'move0',nil,{0.30,-0.25,0.24,0,0,0}},
  {'move0',nil,{0.54,-0.15,0.24,0,0,0}},
}
armfsm.teleop.armuninit={
  {'move0',nil,{0.30,-0.25, 0.24  ,0,0,0}},
  {'move0',nil,{0.25,-0.25,-0.15  ,0,0,0}},
  {'move0',nil,arm.trRArm0},
  {'move0',nil,arm.trRArm0},
}


-- Export
Config.arm = arm
Config.armfsm  = armfsm

return Config
