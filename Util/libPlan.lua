-- libPlan
-- (c) 2014 Stephen McGill
-- Plan a path with the arm
local libPlan = {}
local torch  = require'torch'
torch.Tensor = torch.DoubleTensor
local vector = require'vector'
local q = require'quaternion'
local carray = require 'carray'
local T = require'libTransform'

-- Plan a direct path using
-- a straight line
-- res: resolution in meters
-- TODO: Check some metric of feasibility
local line_path = function(self, qArm, trGoal, res)
	res = res or 0.005
	local K = self.K
	local trArm = K.forward_arm(qArm)
	local qGoal = K.inverse_arm(trGoal,qArm)
	--
	local quatArm, posArm   = T.to_quaternion(trArm)
	local quatGoal, posGoal = T.to_quaternion(trGoal)
	--
	local dPos = posGoal - posArm
	local distance = vector.norm(dPos)
	local nSteps = math.ceil(distance / res)
	local inv_nSteps = 1 / nSteps
	local dTransBack = T.trans(unpack(-inv_nSteps*dPos))

	-- Form the precomputed stack
	local qStack = {}
	local cur_qArm = qGoal
	local cur_trArm = trGoal
	table.insert(qStack,cur_qArm)
	for i=nSteps,1,-1 do
		local trWaypoint = dTransBack * cur_trArm
		local qSlerp = q.slerp(quatArm,quatGoal,i*inv_nSteps)
		local trStep = T.from_quaternion(
			qSlerp,
			{trWaypoint[1][4],trWaypoint[2][4],trWaypoint[3][4]}
		)
		cur_qArm = K.inverse_arm(trStep,cur_qArm)
		cur_trArm = trStep
		table.insert(qStack,cur_qArm)
	end
	return qStack
end

local line_path_iter = function(self, qArm, trGoal, res)
	res = res or 0.005
	local K = self.K
	-- First, find the FK of where we are
	local trArm = K.forward_arm(qArm)
	local quatArm, posArm = T.to_quaternion(trArm)
	-- Second, check the IK of the goal
	local quatGoal, posGoal = T.to_quaternion(trGoal)
	local qGoal = K.inverse_arm(trGoal,qArm)
	-- TODO: Check some metric of feasibility
	local dPos = posGoal - posArm
	local distance = torch.norm(dPos)
	local nSteps = math.ceil(distance / res)
	local inv_nSteps = 1 / nSteps
	--
	local dTransBack = T.trans(-dPos[1]/nSteps,-dPos[2]/nSteps,-dPos[3]/nSteps)

	-- Form the iterator... (more even computation time)
	return function(cur_qArm)
		local cur_trArm = K.forward_arm(cur_qArm)
		local trWaypoint = dTransBack * cur_trArm
		local qSlerp = q.slerp(quatArm,quatGoal,i*inv_nSteps)
		local trStep = T.from_quaternion(
			qSlerp,
			{trWaypoint[1][4],trWaypoint[2][4],trWaypoint[3][4]}
		)
		local iqArm = K.inverse_arm(trStep, cur_qArm)
		return iqArm
	end

end

local joint_path = function(self, qArm, trGoal, res)
end

libPlan.new_planner = function(kinematics, min_q, max_q)
	local planner = {}
	planner.K = kinematics
	planner.min_q = min_q
	planner.max_q = max_q
	planner.line_stack = line_path
	planner.line_iter = line_path_iter
	planner.joint = joint_path
	return planner
end

return libPlan
