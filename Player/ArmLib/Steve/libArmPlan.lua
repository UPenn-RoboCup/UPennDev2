-- libArmPlan
-- (c) 2014 Stephen McGill
-- Plan a path with the arm
local libArmPlan = {}
local procFunc = require'util'.procFunc
local mod_angle = require'util'.mod_angle
local vector = require'vector'
local vnorm = require'vector'.norm
local q = require'quaternion'
local T = require'Transform'
local tremove = require'table'.remove
local tinsert = require'table'.insert
local fabs = require'math'.abs
local min, max = require'math'.min, require'math'.max
local INFINITY = require'math'.huge
local EPSILON = 1e-6

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
			-- don't worry about the yaw ones
			if iq~=5 and iq~=7 then margin = min(fabs(v), margin) end
		end
		for _, v in ipairs(dMax) do
			if iq~=5 and iq~=7 then margin = min(fabs(v), margin) end
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
		local ndp = vnorm(dp) -- NOTE: cost not in joint space
		tinsert(cfk, ndp<IK_POS_ERROR_THRESH and ndp or INFINITY)
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

-- Give a time to complete this
function libArmPlan.joint_preplan(self, qGoal, qArm0, duration)
	assert(type(qGoal)=='table', 'Bad qGoal table')
	assert(#qGoal==self.nq, 'Improper qGoal size')
	assert(type(qArm0)=='table', 'Bad qArm table')
	assert(#qArm0==self.nq, 'Improper qArm size')
	-- Check joint limit compliance
	local qMin, qMax = self.qMin, self.qMax
	for i, q in ipairs(qGoal) do
		assert(q+EPSILON>=qMin[i], string.format('Below qMax[%d] %g < %g', i, q, qMin[i]))
	end
	for i, q in ipairs(qGoal) do
		assert(q-EPSILON<=qMax[i], string.format('Above qMin[%d] %g > %g', i, q, qMax[i]))
	end
	-- Check speed limit compliance
	assert(type(duration)=='number', "Improper duration: "..type(duration))
	local dqTotal = qGoal - qArm0
	local dqdtAverage = dqTotal / duration
	local dqdt_limit = self.dqdt_limit
	for i, dqdt in ipairs(dqdtAverage) do
		assert(fabs(dqdt) <= dqdt_limit[i],
			string.format("Above dqdt[%d] |%g| > %g", i, dqdt, dqdt_limit[i]))
	end
	-- Assume 100 Hz
	local hz = 100
	local dt = 1 / hz
	local nSteps = math.ceil(duration * hz)
	-- Set the path
	local qArm = qArm0
	-- Yield the joint values and % complete
	coroutine.yield(qArm0, 0)
	for i=2, nSteps do
		qArm = qArm + dqdtAverage * dt
		coroutine.yield(qArm, i/nSteps)
	end
	return qGoal, 1
end

local speed_eps = 0.1 * 0.1
local c, p = 2, 10
local torch = require'torch'
-- Use the Jacobian
local function get_delta_qarm(self, vwTarget, qArm)
	-- Penalty for joint limits
	local qMin, qMax, qRange = self.qMin, self.qMax, self.qRange
	local l = {}
	for i, q in ipairs(qArm) do
    l[i]= speed_eps + c * ((2*q - qMin[i] - qMax[i])/qRange[i]) ^ p
  end
	-- Calculate the pseudo inverse
	local J = torch.Tensor(self.jacobian(qArm))
	local JT = J:t():clone()
	local lambda = torch.Tensor(l)
	local Jpseudoinv = torch.mm(torch.inverse(torch.diag(lambda):addmm(JT, J)), JT)

	local null = torch.eye(#l) - Jpseudoinv * J

	local dqArm = torch.mv(Jpseudoinv, torch.Tensor(vwTarget))
	return dqArm, null
end

-- Plan a direct path using a straight line via Jacobian
-- res_pos: resolution in meters
-- res_ang: resolution in radians
function libArmPlan.jacobian_preplan(self, trGoal, qArm0, shoulder_weights)
	local res_pos = self.res_pos
	local res_ang = self.res_ang
	local forward, inverse = self.forward, self.inverse
	local pG = vector.new(T.position(trGoal))
	local invGoal = T.inv(trGoal)
	local qArmFGuess = self:find_shoulder(trGoal, qArm0, shoulder_weights)

	local qArm = qArm0
	coroutine.yield(qArm, 0)
	local done = false
	local n = 0
	repeat
		n = n + 1
		local fkArm = forward(qArm)
		local invArm = T.inv(fkArm)

		local here = invArm * trGoal
		local dp = T.position(here)
		local drpy = T.to_rpy(here)

		local d = vnorm(dp)
		local vwTarget = vector.new{unpack(dp)}
		vwTarget[4], vwTarget[5], vwTarget[6] = unpack(drpy)
		local dqArm, nullspace = self:get_delta_qarm(vwTarget, qArm)
		local dq = vector.new(dqArm)
		local mag = vnorm(dq)
		if mag<2*DEG_TO_RAD then
			break
		elseif mag<5*DEG_TO_RAD then
			qArm = qArm + dq / 10
			--break
		else
			--qArm = qArm + dqArm / 100
			local qnull = nullspace * torch.Tensor(qArm - qArmFGuess)
			--qArm = qArm + dqArm / 100 - vector.new(qnull) / 100
			qArm = qArm + dqArm / 100 - vector.new(qnull) / 100

		end
		coroutine.yield(qArm, 50)
		done = d < 0.02 or n > 1e3
	until done

	--if n>1e3 then print('Jacobian stack is stuck') end
	assert(n<=1e3, 'Jacobian stack is stuck')

	local qArmF = self:find_shoulder(trGoal, qArm, {0,1,0})
	table.insert(qPath, {0, qArmF})

	return qArmF, 1
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
	local nq = 7
	local armOnes = vector.ones(nq)
	local armZeros = vector.zeros(nq)
	local obj = {
		nq = nq,
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
		joint_iter = joint_iter,
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
