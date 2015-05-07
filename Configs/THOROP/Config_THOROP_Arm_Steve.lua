assert(Config, 'Need a pre-existing Config table!')
local vector = require'vector'
local T = require'Transform'
local tr6D = require'Transform'.transform6D

------------------------------------
-- For the arm FSM
local arm = {}

-- Weights: cusage, cdiff, ctight
-- Default init position
arm.trLArm0 = {
	tr=tr6D{0.05, 0.35, -0.25,0,0,0}, options = {5*DEG_TO_RAD}, t=10,
	via='jacobian', weights = {1,0,0}
}
arm.trRArm0 = {
	tr=tr6D{0.05, -0.35, -0.25,0,0,0}, options = {-5*DEG_TO_RAD}, t=10,
	via='jacobian', weights = {1,0,0}
}

-- Default ready position
arm.configL1 = {
	tr=tr6D{0.28, 0.25, 0.2,  0,0,-45*DEG_TO_RAD}, options = {5*DEG_TO_RAD}, t=10,
	via='jacobian', weights = {0,0,1}
}
arm.configR1 = {
	tr=tr6D{0.28, -0.25, 0.2, 0,0,45*DEG_TO_RAD}, options = {-5*DEG_TO_RAD}, t=10,
	via='jacobian', weights = {0,0,1}
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
