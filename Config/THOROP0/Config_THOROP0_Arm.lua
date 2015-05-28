assert(Config, 'Need a pre-existing Config table!')
local vector = require'vector'
local T = require'Transform'

------------------------------------
-- For the arm FSM
-- Weights: cusage, cdiff, ctight, cshoulder, cwrist
local arm = {}

-- This comes after the walkInit
arm.manipulation = {}

arm.manipulation[1] =
	-- Pitch up
	{
	left = {
		via='jacobian_preplan',
		timeout=15,
		tr={0.25, 0.25, 0.15, 0, -60*DEG_TO_RAD,0}, --6D is accepted and converted to tr :)
		qArmGuess = vector.new{135, 0, 0, -135, 90, 45, -90}*DEG_TO_RAD,
		weights = {1,1,-1,1,2},
	},
	right = {
		via='jacobian_preplan',
		timeout=15,
		tr={0.25, -0.25, 0.15, 0, -60*DEG_TO_RAD, 0},
		weights = {0,1,0},
		qArmGuess = vector.new{135, 0, 0, -135, -90, -45, 90}*DEG_TO_RAD,
		weights = {1,1,-1,1,2},
	}
}

-- Now go to yaw
--[[
table.insert(arm.init,
	{
	left = {
		tr={0.25, 0.25, 0.18,    0, 0, -45*DEG_TO_RAD},
		qArmGuess = vector.new{70,25,-28, -150, 10,-80,-90}*DEG_TO_RAD,
		timeout=15,
		via='jacobian_preplan', weights = {0,1,0,1}
	},
	right = {
		tr={0.25, -0.25, 0.18, 0*DEG_TO_RAD, 0*DEG_TO_RAD, 45*DEG_TO_RAD},
		--qArmGuess = vector.new{70,-25,28, -150, 10,-80,-90}*DEG_TO_RAD,
		timeout=15,
		via='jacobian_preplan',
		weights = {0,1,0,1},
	}
})
--]]

arm.pushdoor = {}

arm.pushdoor[1] = {
	left = {
		via='jacobian_preplan',
		timeout=8,
		tr={0.65, 0.25, -0.12, 0, 0*DEG_TO_RAD,0}, --6D is accepted and converted to tr :)
		--via='jacobian_waist_preplan',
		--qWaistGuess = {-10*DEG_TO_RAD,0},
		weights = {1,0,0}
	},
	right = false
}

-- Weights: cusage, cdiff, ctight, cshoulder, cwrist
arm.drill = {}
-- Left views the drill
-- Right grabs the drill
arm.drill[1] = {
	left = {
		timeout=15,
		via='jacobian_preplan',
		tr={0.25, 0.25, 0.3,    0, 0, -90*DEG_TO_RAD},
		qArmGuess = vector.new{0, 45, 90, -90, 0,-45,0}*DEG_TO_RAD,

	 weights = {1,1,-1,1},
	},
	right = {
		timeout=15,
		via='jacobian_preplan',
		tr={0.25, -0.2, 0.3, 0*DEG_TO_RAD, 0*DEG_TO_RAD, 0*DEG_TO_RAD},
		qArmGuess = vector.new{0, -45, -90, -90, 0, 45, 0}*DEG_TO_RAD,
		weights = {1,1,-1,1},
	}
}

arm.shower = {}
arm.shower[1] = {
	left = {
		via='jacobian_preplan',
		timeout=15,
		tr={0.3, 0.35, 0.75, 0, -90*DEG_TO_RAD,0},
		qArmGuess = vector.new{0, 0, 0, -90, 0, 0, 0}*DEG_TO_RAD,
		weights = {1,1,-1,1,2},
	},
	right = {
		via='jacobian_preplan',
		timeout=15,
		tr={0.4, -0.3, 0.7, 0, -80*DEG_TO_RAD, 0},
		qArmGuess = vector.new{0, 0, 0, -90, 0, 0, 0}*DEG_TO_RAD,
		weights = {1,1,-1,1,2},
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
--arm.handoffset.right = offset_ucla_hand

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
armfsm.teleopr = {}
armfsm.teleopr.arminit={
 	{'move0',nil,{0.25,-0.25,-0.15  ,0,0,0}},
  {'move0',nil,{0.30,-0.25,0.24,0,0,0}},
  {'move0',nil,{0.30,-0.10,0.24,0,0,0}},  
}
armfsm.teleopr.armuninit={
  {'move0',nil,{0.30,-0.25, 0.24  ,0,0,0}},
  {'move0',nil,{0.25,-0.25,-0.15  ,0,0,0}},
  {'move0',nil,arm.trRArm0},
}
armfsm.teleopl = {}
armfsm.teleopl.arminit={
 	{'move0',{0.25,0.25,-0.15  ,0,0,0},nil},
--  {'move0',{0.30,0.25,0.24,0,0,0},nil},
--  {'move0',{0.30,0.10,0.24,0,0,0},nil},  
}
armfsm.teleopl.armuninit={
--  {'move0',{0.30,0.25, 0.24  ,0,0,0},nil},
--  {'move0',{0.25,0.25,-0.15  ,0,0,0},nil},
  {'move0',arm.trLArm0,nil},
}


-- Export
Config.arm = arm
Config.armfsm  = armfsm

return Config
