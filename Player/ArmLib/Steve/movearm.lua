local movearm = {}
local Body = require'Body'
local util = require'util'
local vector = require'vector'
local T = require'Transform'
local P = require'libPlan'
local K = require'K_ffi'
local sanitize = K.sanitize

local degreesPerSecond = vector.new{15,10,20, 15, 20,20,20}
local radiansPerSecond = degreesPerSecond * DEG_TO_RAD

local lPlanner = P.new_planner(
	vector.slice(Config.servo.min_rad, Config.parts.LArm[1], Config.parts.LArm[#Config.parts.LArm]),
	vector.slice(Config.servo.max_rad, Config.parts.LArm[1], Config.parts.LArm[#Config.parts.LArm]),
	radiansPerSecond
):set_chain(K.forward_larm, K.inverse_larm)

local rPlanner = P.new_planner(
	vector.slice(Config.servo.min_rad, Config.parts.RArm[1], Config.parts.RArm[#Config.parts.RArm]),
	vector.slice(Config.servo.max_rad, Config.parts.RArm[1], Config.parts.RArm[#Config.parts.RArm]),
	radiansPerSecond
):set_chain(K.forward_rarm, K.inverse_rarm)

-- TODO: Add dt into the joint iterator
local dqLimit = DEG_TO_RAD / 3

-- Take a desired joint configuration and move linearly in each joint towards it
function movearm.goto_q(lwrist, rwrist, safe)
	local lPathIter, rPathIter, iqLArm, iqRArm, qLDist, qRDist
	if lwrist then
		local qLArm = Body.get_larm_command_position()
		lPathIter, iqLArm, qLDist = lPlanner:joint_iter(lwrist, qLArm, dqLimit, safe)
	end
	if rwrist then
		local qRArm = Body.get_rarm_command_position()
		rPathIter, iqRArm, qRDist = rPlanner:joint_iter(rwrist, qRArm, dqLimit, safe)
	end
	return lPathIter, rPathIter, iqLArm, iqRArm, qLDist, qRDist
end

-- Take a desired Transformation matrix and move joint-wise to it
function movearm.goto_tr_via_q(lwrist, rwrist, loptions, roptions)
	local lPathIter, rPathIter, iqLArm, iqRArm, qLDist, qRDist
	if lwrist then
		local qLArm = Body.get_larm_command_position()
		iqLArm = K.inverse_larm(lwrist, qLArm, unpack(loptions or {}))
		lPathIter, iqLArm, qLDist = lPlanner:joint_iter(iqLArm, qLArm, dqLimit, true)
	end
	if rwrist then
		local qRArm = Body.get_rarm_command_position()
		iqRArm = K.inverse_rarm(rwrist, qRArm, unpack(roptions or {}))
		rPathIter, iqRArm, qRDist = rPlanner:joint_iter(iqRArm, qRArm, dqLimit, true)
	end
	return lPathIter, rPathIter, iqLArm, iqRArm, qLDist, qRDist
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
