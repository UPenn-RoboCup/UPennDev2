-- libPlan
-- (c) 2014 Stephen McGill
-- Plan a path with the arm
local libPlan = {}
local torch  = require'torch'
local vector = require'vector'
local q = require'quaternion'
local carray = require 'carray'
local util = require 'util'
--local T = require'libTransform'
local T = require'Transform'

local mt = {}

local function tbl_iter(t,k)
	return table.remove(t)
end
mt.__call  = tbl_iter

local function sanitize(iqWaypoint, cur_qArm, dt, dqdt_limit)
	local diff, mod_diff
	for i, v in ipairs(cur_qArm) do
		diff = iqWaypoint[i] - v
		mod_diff = util.mod_angle(diff)
		if math.abs(diff) > math.abs(mod_diff) then
			if dt then
				iqWaypoint[i] = v + util.procFunc(mod_diff, 0, dqdt_limit[i]*dt)
			else
				iqWaypoint[i] = v + mod_diff
			end
		else
			if dt then
				iqWaypoint[i] = v + util.procFunc(diff, 0, dqdt_limit[i]*dt)
			else
				iqWaypoint[i] = v + diff
			end
		end
	end
end

-- TODO List for the planner
-- TODO: Check some metric of feasibility
-- TODO: Check the FK of the goal so that we are not in the ground, etc.
-- TODO: Check if we must pass through the x,y=0,0
-- to change the orientation through the singularity
-- TODO: Handle goal of transform or just position
-- Just position disables orientation checking

-- Plan a direct path using
-- a straight line
-- res_pos: resolution in meters
-- res_ang: resolution in radians
-- use_safe_inverse: Adjust for pseudo roll?
local line_stack = function(self, qArm, trGoal, res_pos, res_ang, use_safe_inverse)
	res_pos = res_pos or 0.005
	res_ang = res_ang or 1*DEG_TO_RAD
	local K = self.K
	-- If goal is position vector, then skip check
	local skip_angles = type(trGoal[1])=='number'
	-- Save the goal
	local qGoal, posGoal, quatGoal
	if skip_angles==true then
		posGoal = trGoal
		qGoal = K.inverse_arm_position(posGoal,qArm)
	else
		-- Must also fix the rotation matrix, else the yaw will not be correct!
		qGoal = K.inverse_arm(trGoal,qArm)
	end
	--
	qGoal = util.clamp_vector(qGoal,self.min_q,self.max_q)
	--
	local fkGoal = K.forward_arm(qGoal)
	local fkArm  = K.forward_arm(qArm)
	if skip_angles==true then
		posArm = vector.new{fkArm[1][4],fkArm[2][4],fkArm[3][4]}
	else
		quatGoal, posGoal = T.to_quaternion(fkGoal)
		quatArm, posArm   = T.to_quaternion(fkArm)
		vector.new(posGoal)
	end
	--
	local nSteps
	local dPos = posGoal - posArm
	local distance = vector.norm(dPos)
	local nSteps_pos = math.ceil(distance / res_pos)
	if skip_angles==true then
		nSteps = nSteps_pos
	else
		local quatDist = quatArm - quatGoal
		local nSteps_ang = math.ceil(math.abs(quatDist) / res_ang)
		nSteps = math.max(nSteps_pos,nSteps_ang)
	end
	--
	local inv_nSteps = 1 / nSteps
	--local dTransBack = T.trans(unpack(dPos/-nSteps))
	local ddp = dPos/-nSteps
	-- Form the precomputed stack
	local qStack = {}
	local cur_qArm, cur_posArm = vector.copy(qGoal), vector.copy(posGoal)
	--local cur_trArm = trGoal
	--local cur_quatArm = quatArm
	for i=nSteps,1,-1 do
		cur_posArm = cur_posArm + ddp
		if skip_angles==true then
			cur_qArm = K.inverse_arm_position(cur_posArm,cur_qArm)
		else
			local qSlerp = q.slerp( quatArm, quatGoal, i*inv_nSteps )
			local trStep = T.from_quaternion( qSlerp, cur_posArm )
			cur_qArm = K.inverse_arm( trStep, cur_qArm )
		end
		--[[
		local trWaypoint = dTransBack * cur_trArm
		local qSlerp = q.slerp(quatArm,quatGoal,i*inv_nSteps)
		local trStep = T.from_quaternion(
			qSlerp,
			{trWaypoint[1][4],trWaypoint[2][4],trWaypoint[3][4]}
		)
		cur_qArm = K.inverse_arm(trStep,cur_qArm)
		cur_trArm = trStep
		--]]
		table.insert(qStack,cur_qArm)
	end
	-- We return the stack and the final joint configuarion
	return setmetatable(qStack, mt), qGoal
end

-- This should have exponential approach properties...
-- ... are the kinematics "extras" - like shoulderYaw, etc. (Null space)
local line_iter = function(self, trGoal, qArm0, res_pos, res_ang, null_options)
	res_pos = res_pos or 0.005
	res_ang = res_ang or 3*DEG_TO_RAD
	null_options = null_options or {}
	-- If goal is position vector, then skip check
	local skip_angles = type(trGoal[1])=='number'
	--
	local forward, inverse = self.forward, self.inverse
	-- Save the goal
	local qGoal, posGoal, quatGoal
	if skip_angles then
		posGoal = trGoal
		qGoal = inverse_position(posGoal, qArm0, unpack(null_options))
	else
		-- Must also fix the rotation matrix, else the yaw will not be correct!
		qGoal = inverse(trGoal, qArm0, unpack(null_options))
	end
	--
	qGoal = util.clamp_vector(qGoal, self.min_q, self.max_q)
	--
	local fkGoal, null_options0 = forward(qGoal)
	if not skip_angles then
		--print('posGoal In ', trGoal)
		quatGoal, posGoal = T.to_quaternion(fkGoal)
		vector.new(posGoal)
		--print('posGoal Out', posGoal)
	end
	
	local fkArm0, null_options0 = forward(qArm0)
	local quatArm0, posArm0 = T.to_quaternion(fkArm0)
	local dPos0 = posGoal - posArm0
	local distance0 = vector.norm(dPos0)
	
	local dqdt_limit = self.dqdt_limit
	-- We return the iterator and the final joint configuarion
	-- TODO: Add failure detection; if no dist/ang changes in a while
	return function(cur_qArm, dt)
		local cur_trArm, null_options_tmp = forward(cur_qArm)
		--if skip_angles==false and is_singular then print('PLAN SINGULARITY') end
		local trStep, dAng, dAxis, quatArm, posArm
		if skip_angles then
			posArm = vector.new{cur_trArm[1][4], cur_trArm[2][4], cur_trArm[3][4]}
		else
			quatArm, posArm = T.to_quaternion(cur_trArm)
			dAng, dAxis = q.diff(quatArm,quatGoal)
		end
		--
		local dPos = posGoal - vector.new(posArm)
		--print('dPos', dPos, posGoal, posArm)
		local distance = vector.norm(dPos)
		if distance < res_pos then
			if skip_angles or math.abs(dAng)<res_ang or is_singular then
				-- If both within tolerance, then we are done
				-- If singular and no position to go, then done
					-- TODO: Return the goal
				sanitize(qGoal, cur_qArm, dt, dqdt_limit)
				return nil, qGoal, is_singular
			end
			-- Else, just rotate in place
			local qSlerp = q.slerp(quatArm,quatGoal,res_ang/dAng)
			trStep = T.from_quaternion(qSlerp,posGoal)
		elseif skip_angles or math.abs(dAng)<res_ang or is_singular then
			-- Just translation
			local ddpos = (res_pos / distance) * dPos
			if skip_angles then
				return inverse_position(ddpos+posArm, cur_qArm, unpack(null_options))
			end
			trStep = T.trans(unpack(ddpos)) * cur_trArm
		else
			-- Both translation and rotation
			trStep = T.from_quaternion(
				q.slerp(quatArm,quatGoal,res_ang/dAng),
				res_pos * dPos/distance + posArm
			)
		end
		-- Abstract idea of how far we are from the goal, as a percentage.
		-- Closer to the start, means the null options should be closer there, too.
		-- Closer to the finish, the null options should be closer to the goal options
		-- null_options_tmp: has the *current* option
		local null_ph = 1 - math.max(0, math.min(1, distance / distance0))
		for i, v in ipairs(null_options) do
			null_options_tmp[i] = null_options0[i] * (1-null_ph) + v * null_ph
		end
		local iqWaypoint = inverse(trStep, cur_qArm, unpack(null_options_tmp))
		-- Sanitize to avoid trouble with wrist yaw
		sanitize(iqWaypoint, cur_qArm, dt, dqdt_limit)
		return distance, iqWaypoint
	end, qGoal
end

local function joint_stack (self, qGoal, qArm, res_q)
	res_q = res_q or 2*DEG_TO_RAD
	qGoal = util.clamp_vector(qGoal,self.min_q,self.max_q)
	--
	local dq = qGoal - qArm
	local distance = vector.norm(dq)
	local nSteps = math.ceil(distance / res_q)
	local ddq = dq / nSteps
	-- Form the precomputed stack
	local qStack = {}
	for i=nSteps,1,-1 do table.insert(qStack,qArm + i*ddq) end
	-- We return the stack and the final joint configuarion
	return setmetatable(qStack, mt)
end

local function joint_iter(self, qGoal, qArm0, res_q)
	res_q = res_q or 3 * DEG_TO_RAD
	qGoal = vector.copy(util.clamp_vector(qGoal, self.min_q, self.max_q))
	sanitize(qGoal, qArm0, dt, dqdt_limit)
	local dq_min = -res_q * vector.ones(#qGoal)
	local dq_max = res_q * vector.ones(#qGoal)
	local dqdt_limit = self.dqdt_limit
	local qPrev
	return function(cur_qArm, dt)
		-- Feedback
		local dqFB = qGoal - cur_qArm
		local distanceFB = vector.norm(dqFB)
		-- Check if done
		if distanceFB < res_q then return nil, vector.copy(qGoal) end
		local qWaypointFB = cur_qArm + util.clamp_vector(dqFB, dq_min, dq_max)
		sanitize(qWaypointFB, cur_qArm, dt, dqdt_limit)
		-- Feedforward
		qPrev = qPrev or vector.copy(cur_qArm)
		local dqFF = qGoal - qPrev
		local distanceFF = vector.norm(dqFF)
		local qWaypointFF
		if distanceFF < res_q then
			qWaypointFF = vector.copy(qGoal)
		else
			qWaypointFF = qPrev + util.clamp_vector(dqFF, dq_min, dq_max)
		end
		sanitize(qWaypointFF, qPrev, dt, dqdt_limit)
		-- Mix Feedback and Feedforward
		local qWaypoint = 0.25 * qWaypointFB + 0.75 * qWaypointFF
		qPrev = vector.copy(qWaypoint)
		return distanceFB, qWaypoint
	end
end

-- Set the forward and inverse
local function set_chain(self, forward, inverse)
	self.forward = forward
	self.inverse = inverse
end

libPlan.new_planner = function(kinematics, min_q, max_q, dqdt_limit)
	local planner = {
		min_q = min_q or -90*DEG_TO_RAD*vector.ones(7),
		max_q = max_q or 90*DEG_TO_RAD*vector.ones(7),
		dqdt_limit = dqdt_limit or DEG_TO_RAD*vector.new{15,10,10, 15, 20,20,20},
		line_stack = line_stack,
		line_iter = line_iter,
		joint_stack = joint_stack,
		joint_iter = joint_iter,
		-- Default is the left arm:
		inverse = kinematics.inverse_l_arm,
		forward = kinematics.forward_l_arm,
		set_chain = set_chain
	}
	return planner
end

return libPlan
