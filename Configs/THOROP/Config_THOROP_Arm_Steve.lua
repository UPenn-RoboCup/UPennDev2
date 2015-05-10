assert(Config, 'Need a pre-existing Config table!')
local vector = require'vector'
local T = require'Transform'
local tr6D = require'Transform'.transform6D

------------------------------------
-- For the arm FSM
-- Weights: cusage, cdiff, ctight
local arm = {}

arm.init = {}
arm.init[1] = {
	left = {
		tr=tr6D{0.05, 0.35, -0.25,0,0,0}, timeout=10,
		via='jacobian', weights = {1,0,0}
	},
	right = {
		tr=tr6D{0.05, -0.35, -0.25,0,0,0}, timeout=10,
		via='jacobian', weights = {1,0,0}
	}
}

arm.ready = {}
arm.ready[1] = {
	left = {
		tr=tr6D{0.28, 0.25, 0.2,  0,0,-45*DEG_TO_RAD}, timeout=15,
		via='jacobian', weights = {0,0,1}
	},
	right = {
		tr=tr6D{0.28, -0.25, 0.2, 0,0,45*DEG_TO_RAD}, timeout=15,
		via='jacobian', weights = {0,0,1}
	},
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


-- Export
Config.arm = arm

return Config
