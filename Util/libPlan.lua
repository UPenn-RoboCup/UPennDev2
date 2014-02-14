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
	-- Second, check the IK of the goal
	local qGoal = K.inverse_arm(trGoal,qArm)

	-- DEBUG
	print('Start')
	print(T.tostring(trArm))
	print('Goal')
	print(T.tostring(trGoal))
	-- END DEBUG

	-- TODO: Check some metric of feasibility
	local posArm  = trArm:select(2,4):narrow(1,1,3)
	local posGoal = trGoal:select(2,4):narrow(1,1,3)
	local dPos = posGoal - posArm
	local distance = torch.norm(dPos)
	local nSteps = math.ceil(distance / res)
	--[[
	-- Find the position interpolations
	local xpath = torch.range(posArm[1],posGoal[1],dPos[1]/nSteps)
	local ypath = torch.range(posArm[2],posGoal[2],dPos[2]/nSteps)
	local zpath = torch.range(posArm[3],posGoal[3],dPos[3]/nSteps)
	-- TODO: ensure the same size...
	-- Use the position only IK to solve the position path
	local qStack = {}
	local cur_qArm = qGoal
	table.insert(qStack,cur_qArm)
	for i=nSteps,1,-1 do
		local iqArm = K.inverse_arm_pos(
			xpath[i],
			ypath[i],
			zpath[i],
			cur_qArm
		)
		cur_qArm = iqArm
		table.insert(qStack,cur_qArm)
	end
	--]]
	--local dTrans = T.trans(dPos[1]/nSteps,dPos[2]/nSteps,dPos[3]/nSteps)
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
