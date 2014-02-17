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
local line_path_stack = function(self, qArm, trGoal, res)
	res = res or 0.005
	local K = self.K
	local trArm = K.forward_arm(qArm)
	local qGoal = K.inverse_arm(trGoal,qArm)
	local fkGoal = K.forward_arm(qGoal)
	--
	local quatArm,  posArm  = T.to_quaternion(trArm)
	local quatGoal, posGoal = T.to_quaternion(fkGoal)
	--
	local dPos = posGoal - posArm
	local distance = vector.norm(dPos)
	local nSteps = math.ceil(distance / res)
	local inv_nSteps = 1 / nSteps
	local dTransBack = T.trans(unpack(dPos/-nSteps))
	--
	local quatDist = quatArm - quatGoal
	print('Dist (m/deg)',distance,quatDist*RAD_TO_DEG)

	-- Form the precomputed stack
	local qStack = {}
	local cur_qArm = qGoal
	local cur_trArm = trGoal
	table.insert(qStack,cur_qArm)
	for i=nSteps,1,-1 do
		local trWaypoint = dTransBack * cur_trArm
		local qSlerp = q.slerp(quatArm,quatGoal,i*inv_nSteps)

		print('Calc',qSlerp - quatGoal,qSlerp - quatArm)

		local trStep = T.from_quaternion(
			qSlerp,
			{trWaypoint[1][4],trWaypoint[2][4],trWaypoint[3][4]}
		)
		cur_qArm = K.inverse_arm(trStep,cur_qArm)
		cur_trArm = trStep
		table.insert(qStack,cur_qArm)
	end
	table.insert(qStack,qGoal)
	return qStack
end

-- This should hav exponential distance properties...

local line_path_iter = function(self, qArm, trGoal, res)
	res = res or 0.005
	local angle_tolerance = 2*DEG_TO_RAD
	local K = self.K
	local is_close = false
	-- Save the goal
	-- TODO: Check if we must pass through the x,y=0,0
	-- to change the orientation through the singularity
	local qGoal = K.inverse_arm(trGoal,qArm)
	-- Must also fix the rotation matrix, else the yaw will not be correct!
	local fkGoal = K.forward_arm(qGoal)
	local quatGoal, posGoal = T.to_quaternion(fkGoal)
	-- Form the iterator... (more even computation time)
	return function(cur_qArm)
		if is_close==true then return end
		--
		local cur_trArm = K.forward_arm(cur_qArm)
		local quatArm, posArm = T.to_quaternion(cur_trArm)
		--
		local dPos = posGoal - posArm
		local distance = vector.norm(dPos)
		local quatDist = quatArm - quatGoal
		--
		local res_ratio = res / distance
		local res_ratio_ang = angle_tolerance / quatDist
		local trStep
		if distance<res then
			--print(angle_tolerance*RAD_TO_DEG,quatDist*RAD_TO_DEG)
			if quatDist<angle_tolerance then
				is_close = true
				return qGoal
			end
			-- Just rotation (TODO: fix)
			local qSlerp = q.slerp(quatArm,quatGoal,res_ratio_ang)
			trStep = T.from_quaternion(qSlerp,posGoal)
		elseif quatDist<angle_tolerance then
			-- Just translation
			local ddpos = dPos * res / distance
			trStep = T.trans(unpack(ddpos)) * cur_trArm
			q.slerp(quatArm,quatGoal,res_ratio_ang)
		else
			local res_ratio = res / distance
			local res_ratio_ang = angle_tolerance / quatDist
			trStep = T.from_quaternion(
				q.slerp(quatArm,quatGoal,res_ratio_ang),
				dPos*res_ratio + posArm
			)
		end
		return K.inverse_arm(trStep, cur_qArm)
	end

end

local joint_path = function(self, qArm, trGoal, res)
end

libPlan.new_planner = function(kinematics, min_q, max_q)
	local planner = {}
	planner.K = kinematics
	planner.min_q = min_q
	planner.max_q = max_q
	planner.line_stack = line_path_stack
	planner.line_iter = line_path_iter
	planner.joint = joint_path
	return planner
end

return libPlan
