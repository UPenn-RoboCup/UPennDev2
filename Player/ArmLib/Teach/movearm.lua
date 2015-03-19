local movearm = {}
local Body = require'Body'
local T = require'Transform'
local util = require'util'
local vector = require'vector'
local P = require'libPlan'
require'hcm'
local K = require'K_ffi'

local lPlanner = P.new_planner(K,
	vector.slice(Config.servo.min_rad, Config.parts.LArm[1], Config.parts.LArm[#Config.parts.LArm]),
	vector.slice(Config.servo.max_rad, Config.parts.LArm[1], Config.parts.LArm[#Config.parts.LArm]),
	vector.new{15,10,20, 15, 20,20,20}*DEG_TO_RAD
)
lPlanner:set_chain(K.forward_larm, K.inverse_larm)
local rPlanner = P.new_planner(K,
	vector.slice(Config.servo.min_rad, Config.parts.RArm[1], Config.parts.RArm[#Config.parts.RArm]), 
	vector.slice(Config.servo.max_rad, Config.parts.RArm[1], Config.parts.RArm[#Config.parts.RArm]),
	vector.new{15,10,20, 15, 20,20,20}*DEG_TO_RAD -- Angular speedlimits
)
rPlanner:set_chain(K.forward_rarm, K.inverse_rarm)

-- TODO: Add dt into the joint iterator
local dqLimit = DEG_TO_RAD / 3

-- Take a desired Transformation matrix and move joint-wise to it
function movearm.goto_tr_via_q(lwrist, rwrist, loptions, roptions)
	local lPathIter, rPathIter
	if lwrist then
		local qLArm = Body.get_larm_command_position()
		local iqLArm = K.inverse_larm(lwrist, qLArm, unpack(loptions))
		lPathIter = lPlanner:joint_iter(iqLArm, qLArm, dqLimit)
	end
	if rwrist then
		local qRArm = Body.get_rarm_command_position()
		local iqRArm = K.inverse_rarm(rwrist, qRArm, unpack(roptions))
		rPathIter = rPlanner:joint_iter(iqRArm, qRArm, dqLimit)
	end
	return lPathIter, rPathIter
end

-- Take a desired Transformation matrix and move in a line towards it
function movearm.goto_tr(lwrist, rwrist, loptions, roptions)
	local lPathIter, rPathIter
	if lwrist then
		local qLArm = Body.get_larm_command_position()
		lPathIter = lPlanner:line_iter(lwrist, qLArm, 0.01, 3*DEG_TO_RAD, loptions)
	end
	if rwrist then
		local qRArm = Body.get_rarm_command_position()
		rPathIter = rPlanner:line_iter(rwrist, qRArm, 0.01, 3*DEG_TO_RAD, roptions)
	end
	return lPathIter, rPathIter
end

-- Take a desired joint configuration and move linearly in each joint towards it
function movearm.goto_q(lwrist, rwrist)
	local lPathIter, rPathIter
	if lwrist then
		local qLArm = Body.get_larm_command_position()
		lPathIter = lPlanner:joint_iter(lwrist, qLArm, dqLimit)
	end
	if rwrist then
		local qRArm = Body.get_rarm_command_position()
		rPathIter = rPlanner:joint_iter(rwrist, qRArm, dqLimit)
	end
	return lPathIter, rPathIter
end

function movearm.goto_wrists(lwrist, rwrist)
	local lPathIter, rPathIter
	if lwrist then
	  local qLArm = Body.get_larm_position()
	  local qLWrist = Body.get_inverse_lwrist(qLArm, unpack(lwrist, 1, 2))
	  local qLGoal = Body.get_inverse_arm_given_wrist(qLWrist, lwrist[3])
		lPathIter = lPlanner:joint_iter(qLGoal, qLArm, dqLimit)
	end
	if rwrist then
		local qRArm = Body.get_rarm_position()
	  local qRWrist = Body.get_inverse_rwrist(qRArm, unpack(rwrist, 1, 2))
	  local qRGoal = Body.get_inverse_arm_given_wrist(qRWrist, rwrist[3])
		rPathIter = rPlanner:joint_iter(qRGoal, qRArm, dqLimit)
	end
	return lPathIter, rPathIter
end

return movearm
