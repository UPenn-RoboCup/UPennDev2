-- libPlan
-- (c) 2014 Stephen McGill
-- Plan a path with the arm
local libPlan = {}
local torch  = require'torch'
torch.Tensor = torch.DoubleTensor
local vector = require'vector'
local carray = require 'carray'
local T = require'libTransform'

-- Plan a direct path using
-- a straight line
-- res: resolution in meters
local line_path = function(self, qArm, trGoal, res)
	res = res or 0.005
	local K = self.K
	-- First, find the FK of where we are
	local trArm = K.forward_arm(qArm)
	local quatArm, posArm = T.to_quaternion(trArm)
	-- Second, check the IK of the goal
	local quatGoal, posGoal = T.to_quaternion(trGoal)
	local qGoal = K.inverse_arm(trGoal,qArm)

	-- DEBUG
	--[[
	print('Start')
	print(T.tostring(trArm))
	print('Goal')
	print(T.tostring(trGoal))
	--]]
	-- END DEBUG

	-- TODO: Check some metric of feasibility
	local dPos = posGoal - posArm
	local distance = torch.norm(dPos)
	local nSteps = math.ceil(distance / res)
	local dTransBack = T.trans(-dPos[1]/nSteps,-dPos[2]/nSteps,-dPos[3]/nSteps)
	local qStack = {}
	local cur_qArm = qGoal
	local cur_trArm = trGoal
	table.insert(qStack,cur_qArm)
	for i=nSteps,1,-1 do
		local trWaypoint = dTransBack * cur_trArm
		local iqArm = K.inverse_arm(
			trWaypoint,
			cur_qArm
		)
		cur_qArm = iqArm
		cur_trArm = trWaypoint
		table.insert(qStack,cur_qArm)
	end

	-- NOTE: Rotation should immediately happen at the beginning
	-- TODO: This should have some joint interpolation...
	return qStack
end

libPlan.new_planner = function(kinematics, min_q, max_q)
	local planner = {}
	planner.K = kinematics
	planner.min_q = min_q
	planner.max_q = max_q
	planner.line = line_path
	return planner
end

return libPlan
