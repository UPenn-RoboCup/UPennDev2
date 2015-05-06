-- libArmPlan
-- (c) 2014 Stephen McGill
-- Plan a path with the arm
local libArmPlan = {}
local procFunc = require'util'.procFunc
local mod_angle = require'util'.mod_angle
local clamp_vector = require'util'.clamp_vector
local umin = require'util'.min
local vector = require'vector'
local vnorm = require'vector'.norm
local q = require'quaternion'
local T = require'Transform'
local tremove = require'table'.remove
local tinsert = require'table'.insert
local fabs = require'math'.abs
local min, max = require'math'.min, require'math'.max
local INFINITY = require'math'.huge

local function qDiff(iq, q0)
	local diff_use = {}
	local diff, mod_diff
	for i, q in ipairs(q0) do
		diff = iq[i] - q
		mod_diff = mod_angle(diff)
		diff_use[i] = (fabs(diff) > fabs(mod_diff)) and mod_diff or diff
	end
	return diff_use
end

-- No dt needed
local function sanitize0(iqArm, cur_qArm)
	local diff_use = {}
	local mod_diff
	for i, v in ipairs(cur_qArm) do
		diff_use[i] = iqArm[i] - v
		mod_diff = mod_angle(diff_use[i])
		if fabs(diff_use[i]) > fabs(mod_diff) then
			iqArm[i] = v + mod_diff
			diff_use[i] = mod_diff
		end
	end
	return diff_use
end

-- Requires dt
local function sanitize(iqWaypoint, cur_qArm, dt, dqdt_limit)
	local diff_use = {}
	local diff0, mod_diff, diff
	for i, v in ipairs(cur_qArm) do
		diff0 = iqWaypoint[i] - v
		mod_diff = mod_angle(diff0)
		-- Use the smaller diff
		diff = fabs(diff0) > fabs(mod_diff) and mod_diff or diff0
		diff_use[i] = procFunc(diff, 0, dqdt_limit[i] * dt)
		iqWaypoint[i] = v + diff_use[i]
	end
	return diff_use
end

local mt = {
	__call = function(t, q, dt)
		local nxt = t[#t]
		if not nxt then return end
		if t.done_wp then
			t.done_wp = false
			local dist, wp = unpack(tremove(t))
			return dist, wp
		elseif t.dqdt_limit then
			local dist, wp = unpack(nxt)
			local blend_wp = vector.copy(wp)
			local diff_use = sanitize(blend_wp, q, dt, t.dqdt_limit)
			t.done_wp = vnorm(diff_use) < 0.001
			return dist, t.done_wp and sanitize0(wp, q) or blend_wp
		else
			return unpack(tremove(t))
		end
	end
}

-- Similar to SJ's method for the margin
local function find_shoulder_sj(self, tr, qArm)
	local minArm, maxArm, rangeArm = self.qMin, self.qMax, self.qRange
	local invArm = self.inverse
	local iqArm, margin, qBest
	local dMin, dMax
	--
	local maxmargin, margin = -INFINITY
	for i, q in ipairs(self.shoulderAngles) do
		iqArm = invArm(tr, qArm, q)
		-- Maximize the minimum margin
		dMin = iqArm - minArm
		dMax = iqArm - maxArm
		margin = INFINITY
		for _, v in ipairs(dMin) do
			if iq~=5 and iq~=7 then -- don't worry about the yaw ones
				margin = min(fabs(v), margin)
			end
		end
		for _, v in ipairs(dMax) do
			if iq~=5 and iq~=7 then
				margin = min(fabs(v), margin)
			end
		end
		if margin > maxmargin then
			maxmargin = margin
			qBest = iqArm
		end
	end
	return qBest
end



local IK_POS_ERROR_THRESH = 0.0254
-- Weights: cusage, cdiff, ctight
local defaultWeights = {1, 1, 0}
local function valid_cost(iq, minArm, maxArm)
	for i, q in ipairs(iq) do
		if q<minArm[i] or q>maxArm[i] then return INFINITY end
	end
	return 0
end
local function find_shoulder(self, tr, qArm, weights)
	weights = weights or defaultWeights
	-- Form the inverses
	local iqArms = {}
	for _, q in ipairs(self.shoulderAngles) do
		local iq = self.inverse(tr, qArm, q)
		local du = sanitize0(iq, qArm)
		tinsert(iqArms, iq)
	end
	-- Form the FKs
	local fks = {}
	for ic, iq in ipairs(iqArms) do fks[ic] = self.forward(iq) end
	--
	local minArm, maxArm = self.qMin, self.qMax
	local rangeArm, halfway = self.qRange, self.halves
	-- Cost for valid configurations
	local cvalid = {}
	for ic, iq in ipairs(iqArms) do tinsert(cvalid, valid_cost(iq, minArm, maxArm) ) end
	-- FK sanity check
	local dps = {}
	local p_tr = vector.new(T.position(tr))
	for i, fk in ipairs(fks) do dps[i] = p_tr - T.position(fk) end
	local cfk = {}
	for ic, dp in ipairs(dps) do
		tinsert(cfk, vnorm(dp)<IK_POS_ERROR_THRESH and 0 or INFINITY)
	end
	-- Minimum Difference in angles
	local cdiff = {}
	for ic, iq in ipairs(iqArms) do
		--tinsert(cdiff, fabs(iq[3] - qArm[3]))
		tinsert(cdiff, vnorm(qDiff(iq, qArm)))
	end
	-- Cost for being tight (Percentage)
	local ctight = {}
	-- The margin from zero degrees away from the body
	local margin, ppi = 5*DEG_TO_RAD, math.pi
	for _, iq in ipairs(iqArms) do tinsert(ctight, fabs((iq[2]-margin))/ppi) end
	-- Usage cost (Worst Percentage)
	local cusage, dRelative = {}
	for _, iq in ipairs(iqArms) do
		dRelative = ((iq - minArm) / rangeArm) - halfway
		-- Don't use the infinite yaw ones
		tremove(dRelative, 7)
		tremove(dRelative, 5)
		-- Don't use a cost based on the shoulderAngle
		tremove(dRelative, 3)
		tinsert(cusage, max(fabs(min(unpack(dRelative))), fabs(max(unpack(dRelative)))))
	end
	-- Combined cost
	-- TODO: Tune the weights on a per-task basis (some tight, but not door)
	local cost = {}
	for ic, valid in ipairs(cvalid) do
		tinsert(cost, valid + cfk[ic]
			+ weights[1]*cusage[ic] + weights[2]*cdiff[ic] + weights[3]*ctight[ic])
	end
	-- Find the smallest cost
	local ibest, cbest = 0, INFINITY
	for i, c in ipairs(cost) do if c<cbest then cbest = c; ibest = i end end
	-- Yield the least cost arms
	assert(iqArms[ibest], 'find_shoulder | No solution')
	return iqArms[ibest], fks[ibest]
end

-- TODO List for the planner
-- TODO: Check some metric of feasibility
-- TODO: Check the FK of the goal so that we are not in the ground, etc.
-- TODO: Check if we must pass through the x,y=0,0
-- to change the orientation through the singularity
-- TODO: Handle goal of transform or just position
-- Just position disables orientation checking

-- This should have exponential approach properties...
-- ... are the kinematics "extras" - like shoulderYaw, etc. (Null space)
local function line_iter(self, trGoal, qArm0, null_options, shoulder_weights)

	local dqdt_limit = self.dqdt_limit
	local res_pos = self.res_pos
	local res_ang = self.res_ang
	--
	local forward, inverse = self.forward, self.inverse
	-- Save the goal
	local qGoal
	-- Must also fix the rotation matrix, else the yaw will not be correct!
	if null_options then
		qGoal = inverse(trGoal, qArm0, unpack(null_options))
		sanitize0(qGoal, qArm0)
	else
		qGoal = find_shoulder(self, trGoal, qArm0, shoulder_weights)
		-- TODO: set null_options
	end
	assert(qGoal, 'line_iter | No valid qGoal!')
	local fkGoal = forward(qGoal)
	local quatGoal, posGoal = T.to_quaternion(fkGoal)
	vector.new(posGoal)
	--
	local fkArm0, null_options0 = forward(qArm0)
	local quatArm0, posArm0 = T.to_quaternion(fkArm0)
	local dPos0 = posGoal - posArm0
	local distance0 = vnorm(dPos0)

	-- We return the iterator and the final joint configuarion
	-- TODO: Add failure detection; if no dist/ang changes in a while
	return function(cur_qArm, dt)
		local cur_trArm = forward(cur_qArm)
		local trStep
		local quatArm, posArm = T.to_quaternion(cur_trArm)
		local dAng, dAxis = q.diff(quatArm,quatGoal)
		--
		local dPos = posGoal - vector.new(posArm)
		--print('dPos', dPos, posGoal, posArm)
		local distance = vnorm(dPos)
		if distance < res_pos then
			if fabs(dAng)<res_ang or is_singular then
				-- If both within tolerance, then we are done
				-- If singular and no position to go, then done
					-- TODO: Return the goal
				if dt then
					sanitize(qGoal, cur_qArm, dt, dqdt_limit)
				else
					sanitize0(qGoal, cur_qArm)
				end
				return false, qGoal, is_singular
			end
			-- Else, just rotate in place
			local qSlerp = q.slerp(quatArm,quatGoal,res_ang/dAng)
			trStep = T.from_quaternion(qSlerp,posGoal)
		elseif fabs(dAng)<res_ang or is_singular then
			-- Just translation
			local ddpos = (res_pos / distance) * dPos
			ddpos = distance < 2*res_pos and ddpos / 2 or ddpos
			trStep = T.trans(unpack(ddpos)) * cur_trArm
		else
			local ddrot = res_ang/dAng
			ddrot =  dAng < 2*res_ang and ddrot/2 or ddrot
			local ddpos = (res_pos / distance) * dPos
			ddpos = distance < 2*res_pos and ddpos / 2 or ddpos
			-- Both translation and rotation
			trStep = T.from_quaternion(
				q.slerp(quatArm,quatGoal,ddrot),
				ddpos + posArm
			)
		end
		-- Abstract idea of how far we are from the goal, as a percentage.
		-- Closer to the start, means the null options should be closer there, too.
		-- Closer to the finish, the null options should be closer to the goal options
		-- null_options_tmp: has the *current* option
		local iqWaypoint
		--print(null_options)
		if null_options then
			local null_options_tmp = {}
			local null_ph = 1 - max(0, min(1, distance / distance0))
			--[[
			for i, v in ipairs(null_options) do
				null_options_tmp[i] = null_options0[i] * (1-null_ph) + v * null_ph
			end
			--]]
			local shoulderBlend = null_options0[1] * (1-null_ph) + null_options[1] * null_ph
			iqWaypoint = inverse(trStep, cur_qArm, shoulderBlend, null_options0[2])
		else
			iqWaypoint = find_shoulder(self, trStep, cur_qArm, shoulder_weights)
			assert(iqWaypoint, 'line_iter | No valid iqWaypoint!')
		end
		-- Sanitize to avoid trouble with wrist yaw
		if dt then
			sanitize(iqWaypoint, cur_qArm, dt, dqdt_limit)
		else
			sanitize0(iqWaypoint, cur_qArm)
		end
		return distance, iqWaypoint
	end, qGoal, distance0
end

-- TODO: Optimize the feedforward ratio
local function joint_iter(self, qGoal0, qArm0, SANITIZE)
	local res_ang = self.res_ang
	local dq_max = res_ang * self.ones
	local dq_min = -1 * dq_max
	local qGoal = clamp_vector(qGoal0, self.qMin, self.qMax)
	local dqdt_limit = self.dqdt_limit
	if SANITIZE then sanitize0(qGoal, qArm0) end
	local distance0 = vnorm(qGoal - qArm0)
	local qPrev
	return function(cur_qArm, dt)
		-- Feedback
		local dqFB = qGoal - cur_qArm
		local distanceFB = vnorm(dqFB)
		-- Check if done
		if distanceFB < res_ang then return nil, qGoal end
		local qWaypointFB = cur_qArm + clamp_vector(dqFB, dq_min, dq_max)
		if dt then
			sanitize(qWaypointFB, cur_qArm, dt, dqdt_limit)
		else
			sanitize0(qWaypointFB, cur_qArm)
		end
		-- Feedforward
		qPrev = qPrev or cur_qArm
		local dqFF = qGoal - cur_qArm --qPrev
		local distanceFF = vnorm(dqFF)
		local qWaypointFF
		if distanceFF < res_ang then
			qWaypointFF = qGoal
		else
			qWaypointFF = qPrev + clamp_vector(dqFF, dq_min, dq_max)
			if dt then
				sanitize(qWaypointFF, qPrev, dt, dqdt_limit)
			else
				sanitize0(qWaypointFF, qPrev)
			end
		end
		-- Mix Feedback and Feedforward
		local qWaypoint = 0.2 * qWaypointFB + 0.8 * qWaypointFF
		qPrev = qWaypoint
		return distanceFB, qWaypoint
	end, qGoal, distance0
end

-- Plan a direct path using
-- a straight line
-- res_pos: resolution in meters
-- res_ang: resolution in radians
local function line_stack(self, trGoal, qArm0, null_options, shoulder_weights)
	local res_pos = self.res_pos
	local res_ang = self.res_ang
	local forward, inverse = self.forward, self.inverse
	-- Must also fix the rotation matrix, else the yaw will not be correct!
	--local qGoal = inverse(trGoal, qArm0)
	local qGoal = find_shoulder(self, trGoal, qArm0, shoulder_weights)
	--
	--qGoal = clamp_vector(qGoal,self.qMin,self.qMax)
	sanitize0(qGoal, qArm0)
	--
	local fkGoal = forward(qGoal)
	local fkArm  = forward(qArm0)

	local quatGoal, posGoal = T.to_quaternion(fkGoal)
	local quatArm, posArm   = T.to_quaternion(fkArm)
	vector.new(posGoal)

	--
	local nSteps
	local dPos = posGoal - posArm
	local distance = vnorm(dPos)
	local nSteps_pos = math.ceil(distance / res_pos)
	local quatDist = quatArm - quatGoal
	local nSteps_ang = math.ceil(fabs(quatDist) / res_ang)
	nSteps = max(nSteps_pos,nSteps_ang)
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
		local qSlerp = q.slerp( quatArm, quatGoal, i*inv_nSteps )
		local trStep = T.from_quaternion( qSlerp, cur_posArm )
		--cur_qArm = inverse( trStep, cur_qArm )
		cur_qArm = find_shoulder(self, trStep, cur_qArm, shoulder_weights)
		--[[
		local trWaypoint = dTransBack * cur_trArm
		local qSlerp = q.slerp(quatArm,quatGoal,i*inv_nSteps)
		local trStep = T.from_quaternion(
			qSlerp,
			{trWaypoint[1][4],trWaypoint[2][4],trWaypoint[3][4]}
		)
		cur_qArm = inverse(trStep,cur_qArm)
		cur_trArm = trStep
		--]]
		--sanitize0(cur_qArm, qArm0)
		--sanitize0(cur_qArm, qGoal)
		--print('cur_qArm', cur_qArm)
		table.insert(qStack, {vnorm(ddp*i), vector.new(cur_qArm)})
	end
	qStack.dqdt_limit = self.dqdt_limit
	-- We return the stack and the final joint configuarion
	return setmetatable(qStack, mt), qGoal, distance
end

local function joint_stack(self, qGoal, qArm)
	local res_ang = self.res_ang
	qGoal = clamp_vector(qGoal,self.qMin,self.qMax)
	--
	local dq = qGoal - qArm
	local distance = vnorm(dq)
	local nSteps = math.ceil(distance / res_ang)
	local ddq = dq / nSteps
	-- Form the precomputed stack
	local qStack = {}
	for i=nSteps,1,-1 do table.insert(qStack,qArm + i*ddq) end
	-- We return the stack and the final joint configuarion
	return setmetatable(qStack, mt)
end

local speed_eps = 0.1 * 0.1
local c, p = 2, 10
local torch = require'torch'
-- Use the Jacobian
local function get_delta_qarm(self, vwTarget, qArm)
	-- Penalty for joint limits
	local qMin, qMax, qRange = self.qMin, self.qMax, self.qRange
	local lambda = {}
	for i, q in ipairs(qArm) do
    lambda[i]= speed_eps + c * ((2*q - qMin[i] - qMax[i])/qRange[i]) ^ p
  end
	-- Calculate the pseudo inverse
	local J = torch.Tensor(self.jacobian(qArm))
	local JT = J:t():clone()
	local I = torch.diag(torch.Tensor(lambda)):addmm(JT, J)
	local Iinv = torch.inverse(I)
	local I2 = torch.mm(Iinv, JT)
	local e = torch.Tensor(vwTarget)
	local dqArm = torch.mv(I2, e)
	return vector.new(dqArm)
end

-- Plan a direct path using a straight line via Jacobian
-- res_pos: resolution in meters
-- res_ang: resolution in radians
local function jacobian_stack(self, trGoal, qArm0, null_options, shoulder_weights)
	local res_pos = self.res_pos
	local res_ang = self.res_ang
	local forward, inverse = self.forward, self.inverse
	local pG = vector.new(T.position(trGoal))
	local qStack = {}
	local invGoal = T.inv(trGoal)

	local qArm = qArm0
	local done = false
	local n = 0
	repeat
		n = n + 1
		local fkArm = forward(qArm)
		local invArm = T.inv(fkArm)

		--local p = vector.new(T.position(fkArm))
		--local dp = pG - p

		local here = invArm*trGoal
		local dp = T.position(here)
		local drpy = T.to_rpy(here)
		--print(unpack(dp))
		--print(vector.new(drpy)*RAD_TO_DEG)

		local d = vnorm(dp)
		local vwTarget = vector.new{unpack(dp)}
		vwTarget[4], vwTarget[5], vwTarget[6] = unpack(drpy)
		local dqArm = self:get_delta_qarm(vwTarget, qArm)
		local mag = vnorm(dqArm)
		--print(mag)
		if mag<3*DEG_TO_RAD then
			qArm = qArm + dqArm
			break
		else
			qArm = qArm + dqArm / 100
		end

		--[[
		print('--')
		print(unpack(T.position(invGoal*fkArm)))
		print(unpack(T.position(fkArm*invGoal)))
		print(unpack(T.position(invArm*trGoal))) -- good
		print(unpack(T.position(trGoal*invArm)))
		print(dp)
		--]]

		--[[
		if self.id=='Right' then
			--print('dqArm', dqArm / 20 * RAD_TO_DEG)
			print(vector.new(dp), d, pG)
		--print(unpack(dp))
		--print(vector.new(drpy)*RAD_TO_DEG)
			--print('dist', d, p - pG)
		end
		--]]

		table.insert(qStack, {d, qArm})
		done = d < 0.02 or n > 1e3
	until done

	print('qStack', n)



	-- Reverse
	local qStack2 = setmetatable({}, mt)
	for i=#qStack, 1, -1 do
		table.insert(qStack2, qStack[i])
	end
	local qArmF = self:find_shoulder(trGoal, qStack2[1][2], {0,1,0})

	--qStack2.dqdt_limit = self.dqdt_limit
	return qStack2, qStack2[1][2], 1
end









-- Set the forward and inverse
local function set_chain(self, forward, inverse, jacobian)
	self.forward = assert(forward)
	self.inverse = assert(inverse)
	self.jacobian = jacobian
  return self
end

-- Set the iterator resolutions
local function set_resolution(self, res_pos, res_ang)
	self.res_pos = assert(res_pos)
	self.res_ang = assert(res_ang)
  return self
end

-- Set the iterator resolutions
local function set_limits(self, qMin, qMax, dqdt_limit)
	self.qMin = assert(qMin)
	self.qMax = assert(qMax)
	self.dqdt_limit = assert(dqdt_limit)
  return self
end

local function set_shoulder_angles(self, granularity)
	local granularity = 2*DEG_TO_RAD
	local minShoulder, maxShoulder = self.qMin[3], self.qMax[3]
	local n = math.floor((maxShoulder - minShoulder) / granularity + 0.5)
	local shoulderAngles = {}
	for i=0,n do table.insert(shoulderAngles, minShoulder + i * granularity) end
	self.shoulderAngles = shoulderAngles
	return self
end

-- Still must set the forward and inverse kinematics
function libArmPlan.new_planner(qMin, qMax, dqdt_limit, res_pos, res_ang)
	local armOnes = vector.ones(7)
	local armZeros = vector.zeros(7)
	local obj = {
		forward = nil,
		inverse = nil,
		--
		zeros = armZeros,
		ones = armOnes,
		halves = armOnes * 0.5,
		--
		qMin = qMin or -90*DEG_TO_RAD*armOnes,
		qMax = qMax or 90*DEG_TO_RAD*armOnes,
		dqdt_limit = dqdt_limit or 20*DEG_TO_RAD*armOnes,
		--
		res_pos = res_pos or 0.01,
		res_ang = res_ang or 2*DEG_TO_RAD,
		shoulderAngles = nil,
		--
		line_stack = line_stack,
		line_iter = line_iter,
		joint_stack = joint_stack,
		joint_iter = joint_iter,
		jacobian_stack = jacobian_stack,
		--
		get_delta_qarm = get_delta_qarm,
		--
		find_shoulder = find_shoulder,
		--
		set_chain = set_chain,
		set_resolution = set_resolution,
		set_limits = set_limits,
	}
	obj.qRange = obj.qMax - obj.qMin
	-- Setup the shoulder angles
	set_shoulder_angles(obj)
	--
	return obj
end

return libArmPlan
