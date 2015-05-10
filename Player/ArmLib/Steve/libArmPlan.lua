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
local EPSILON = 1e-3 * DEG_TO_RAD

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
local defaultWeights = {1, 0, 0}
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
	return iqArms[ibest], fks[ibest]
end

function libArmPlan.joint_preplan(self, qArm0, plan)
	assert(type(plan)=='table', 'joint_preplan | Bad plan')
	local timeout = assert(plan.timeout, 'joint_preplan | No timeout')
	local weights = plan.weights
	local qArmF = plan.q
	if plan.tr then
		qArmF = self:find_shoulder(plan.tr, qArm0, weights)
	end
	assert(qArmF, 'joint_preplan | No target shoulder solution')
	local duration = plan.duration
	local hz, dt = self.hz, self.dt
	local dq_limit = self.dq_limit
	local qMin, qMax = self.qMin, self.qMax
	assert(type(qArmF)=='table', 'joint_preplan | Bad qGoal table')
	assert(#qArmF==self.nq, 'joint_preplan | Improper qGoal size')
	assert(type(qArm0)=='table', 'joint_preplan | Bad qArm table')
	assert(#qArm0==self.nq, 'joint_preplan | Improper qArm size')
	-- Check joint limit compliance
	for i, q in ipairs(qArmF) do
		assert(q+EPSILON>=qMin[i], string.format('joint_preplan | Below qMax[%d] %g < %g', i, q, qMin[i]))
	end
	for i, q in ipairs(qArmF) do
		assert(q-EPSILON<=qMax[i], string.format('joint_preplan | Above qMin[%d] %g > %g', i, q, qMax[i]))
	end

	-- Set the timeout
	assert(type(timeout)=='number', "joint_preplan | Improper timeout: "..type(timeout))
	local nStepsTimeout = timeout * hz

	-- If given a duration, then check speed limit compliance
	local dqAverage
	if type(duration)=='number' then
		print('Using duration')
		assert(timeout>=duration,
			string.format('joint_preplan | Timeout %g < Duration %g', timeout, duration))
		local dqTotal = qArmF - qArm0
		local dqdtAverage = dqTotal / duration
		dqAverage = dqdtAverage * dt
		for i, lim in ipairs(dq_limit) do
			assert(fabs(dqAverage[i]) <= lim,
				string.format("joint_preplan | dq[%d] |%g| > %g", i, dqAverage[i], lim))
		end
		nStepsTimeout = duration * hz
	end


	local path = {}
	local qArm = qArm0
	n = 0
	repeat
		n = n + 1
		local dqArmF = qArmF - qArm
		local dist = vnorm(dqArmF)
		local dqUsed
		if dqAverage then
			dqUsed = dqAverage
		else
			-- Check the speed limit usage
			local usage, rescale = {}, false
			for i, limit in ipairs(dq_limit) do
				-- half speed?
				local use = fabs(dqArmF[i]) / limit
				rescale = rescale or use > 1
				table.insert(usage, use)
			end
			if rescale then
				local max_usage2 = max(unpack(usage))
				--print('Rescaling 2!', max_usage2)
				for i, qF in ipairs(dqArmF) do
					dqArmF[i] = qF / max_usage2
				end
			end
			dqUsed = dqArmF
		end
		-- Apply the joint change
		qArm = qArm + dqUsed
		--print(qArm, dqUsed)
		table.insert(path, qArm)

		--print('final dist', dist*RAD_TO_DEG)
		if (not dqAverage) and (dist < 0.5*DEG_TO_RAD) then break end
	until n > nStepsTimeout


	print(n, 'joint_preplan steps')
	assert(dqAverage or (n <= nStepsTimeout),
		'joint_preplan | Timeout: '..nStepsTimeout)

	local qArmSensed = coroutine.yield(qArmFGuess, dist_components)
	-- Progress is different, now, since in joint space
	for i, qArmPlanned in ipairs(path) do
		qArmSensed = coroutine.yield(qArmPlanned)
		-- Check the lage
		local dqLag = qArm - qArmSensed
		local imax_lag, max_lag = 0, 0
		for i, dq in ipairs(dqLag) do
			--local lag = fabs(dq-dqUsed[i])
			--assert(lag<3*DEG_TO_RAD, 'joint_preplan2 | Bad Final Lag: '..tostring(lag))
		end
	end
	return dqArmF
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

local function get_distance(self, qArm, trGoal)
	-- Grab our relative transform from here to the goal
	local fkArm = self.forward(qArm)
	local invArm = T.inv(fkArm)
	local here = invArm * trGoal
	-- Determine the position and angular velocity target
	local dp = T.position(here)
	local drpy = T.to_rpy(here)
	local components = {vnorm(dp), vnorm(drpy)}
	return dp, drpy, components
end

-- Plan a direct path using a straight line via Jacobian
-- res_pos: resolution in meters
-- res_ang: resolution in radians
function libArmPlan.jacobian_preplan(self, qArm0, plan)
	assert(type(plan)=='table', 'jacobian_preplan | Bad plan')
	assert(plan.tr or plan.q, 'jacobian_preplan | Need tr or q')
	local trGoal = plan.tr or self.forward(plan.q)
	local timeout = assert(plan.timeout, 'jacobian_preplan | No timeout')
	local weights = plan.weights

	local hz, dt = self.hz, self.dt
	local dq_limit = self.dq_limit
	local qMin, qMax = self.qMin, self.qMax

	-- What is the weight of the null movement?
	--local alpha_n = 0.5
	-- Find a guess of the final arm position
	local qArmFGuess = self:find_shoulder(trGoal, qArm0, weights)
	--assert(qArmFGuess, 'jacobian_preplan | No guess shoulder solution')
	qArmFGuess = qArmFGuess or qArm0
	local qArm = qArm0
	--print('get_distance')
	local dp, drpy, dist_components = get_distance(self, qArm, trGoal)
	--local dp0, drpy0, dist_components0 = dp, drpy, dist_components
	--print('dist_components', unpack(dist_components))
	--print('qArmSensed', qArmSensed)

	local t0 = unix.time()
	local nStepsTimeout = math.ceil(timeout * hz)
	local done = false
	local n = 0
	local max_usage
	local path = {}
	repeat
		n = n + 1
		local vwTarget = {unpack(dp)}
		vwTarget[4], vwTarget[5], vwTarget[6] = unpack(drpy)
		-- Grab the joint velocities needed to accomplish the se(3) velocities
		local dqdtArm, nullspace = get_delta_qarm(self, vwTarget, qArm)
		-- Grab the null space velocities toward our guessed configuration
		local dqdtNull = nullspace * torch.Tensor(qArm - qArmFGuess)
		-- Linear combination of the two
		--local dqdtCombo = dqdtArm:mul(1-alpha_n) - dqdtNull:mul(alpha_n)
		local dqdtCombo = dqdtArm - dqdtNull
		-- Respect the update rate, place as a lua table
		local dqCombo = vector.new(dqdtCombo:mul(dt))
		-- Check the speed limit usage
		local usage, rescale = {}, false
		for i, limit in ipairs(dq_limit) do
			local use = fabs(dqCombo[i]) / limit
			rescale = rescale or use > 1
			table.insert(usage, use)
		end
		max_usage = max(unpack(usage))
		if rescale then
			--print('Rescaling!', max_usage)
			for i = 1, #dqCombo do
				dqCombo[i] = dqCombo[i] / max_usage
			end
		end
		-- Apply the joint change
		qArm = qArm + dqCombo
		-- Check joint limit compliance
		for i, q in ipairs(qArm) do qArm[i] = min(max(qMin[i], q), qMax[i]) end
		-- Yield the progress
		dp, drpy, dist_components = get_distance(self, qArm, trGoal)
		--print('dist_components', unpack(dist_components))

		table.insert(path, qArm)

		-- Check if we are close enough
		if dist_components[1] < 0.01 and dist_components[2] < 2*RAD_TO_DEG then
			break
		end
	until n > nStepsTimeout
	local t1 = unix.time()
	print(n, 'jacobian steps planned in: ', t1-t0)

	local qArmSensed = coroutine.yield(qArmFGuess, dist_components)

	for i, qArmPlanned in ipairs(path) do
		qArmSensed = coroutine.yield(qArmPlanned)
		-- If we are lagging badly, then there may be a collision
		local dqLag = qArmPlanned - qArmSensed
		local imax_lag, max_lag = 0, 0
		for i, dq in ipairs(dqLag) do
			--print(dq, dqCombo[i])
			--print((dq-dqCombo[i])*RAD_TO_DEG)
			--local lag = fabs(dq-dqCombo[i])
			--assert(lag<3*DEG_TO_RAD, 'jacobian_preplan | Bad Lag: '..tostring(lag))
		end
	end

	assert(n <= nStepsTimeout, 'jacobian_preplan | Timeout')
	local qArmF = self:find_shoulder(trGoal, qArm, {0,1,0})
	--assert(qArmF, 'jacobian_preplan | No final shoulder solution')
	qArmF = qArmF or qArm

	-- Goto the final arm position as quickly as possible
	-- NOTE: We assume the find_shoulder yields a valid final configuration
	-- Use the last known max_usage to finalize
	print('max_usage final', max_usage)
	n = 0
	nStepsTimeout = 3 * hz -- 3 second timeout to finish
	repeat
		n = n + 1
		local dqArmF = qArmF - qArm
		local dist = vnorm(dqArmF)
		-- Check the speed limit usage
		local usage, rescale = {}, false
		for i, limit in ipairs(dq_limit) do
			-- half speed?
			local use = fabs(dqArmF[i]) / (limit*max_usage)
			rescale = rescale or use > 1
			table.insert(usage, use)
		end
		if rescale then
			local max_usage2 = max(unpack(usage))
			--print('Rescaling 2!', max_usage2)
			for i, qF in ipairs(dqArmF) do
				dqArmF[i] = qF / max_usage2
			end
		end
		-- Apply the joint change
		qArm = qArm + dqArmF
		-- Progress is different, now, since in joint space
		qArmSensed = coroutine.yield(qArm, dqArmF)
		-- Check the lage
		local dqLag = qArm - qArmSensed
		local imax_lag, max_lag = 0, 0
		for i, dq in ipairs(dqLag) do
			local lag = fabs(dq-dqArmF[i])
			--assert(lag<3*DEG_TO_RAD, 'jacobian_preplan | Bad Final Lag: '..tostring(lag))
		end
		--print('final dist', dist*RAD_TO_DEG)
		if dist < 0.5*DEG_TO_RAD then break end
	until n > nStepsTimeout
	print(n, 'final steps')
	assert(n <= nStepsTimeout, 'jacobian_preplan | Final timeout')

	return qArmF
end

-- resume with: qArmSensed, vwTargetNew, weightsNew
function libArmPlan.jacobian_velocity(self, qArm0, plan)
	assert(type(plan)=='table', 'Bad plan')
	local vwTarget = assert(plan.vwTarget)
	local weights = assert(plan.weights)
	local timeout = assert(plan.timeout)

	local hz, dt = self.hz, self.dt
	local dq_limit = self.dq_limit
	local qMin, qMax = self.qMin, self.qMax
	local forward = self.forward

	-- 3 second default timeout
	local nStepsTimeout = math.ceil(timeout * hz)
	-- Find a guess of the final arm position
	local fkArm0 = forward(qArm0)
	local qArmFGuess = self:find_shoulder(fkArm0, qArm0, weights) or qArm0
	local qArm = qArm0

	local qArmSensed, vwTargetNew, weightsNew, qArmFGuessNew =
		coroutine.yield(nStepsTimeout)
	vwTarget = vwTargetNew or vwTarget
	weights = weightsNew or weights
	qArmFGuess = qArmFGuessNew or qArmFGuess
	local done = false
	local n = 0
	local max_usage
	repeat
		n = n + 1
		-- Grab the joint velocities needed to accomplish the se(3) velocities
		local dqdtArm, nullspace = get_delta_qarm(self, vwTarget, qArmSensed)
		-- Grab the null space velocities toward our guessed configuration
		local dqdtNull = nullspace * torch.Tensor(qArm - qArmFGuess)
		-- Linear combination of the two
		local dqdtCombo = dqdtArm - dqdtNull
		-- Respect the update rate, place as a lua table
		local dqCombo = vector.new(dqdtCombo:mul(dt))
		-- Check the speed limit usage
		local usage, rescale = {}, false
		for i, limit in ipairs(dq_limit) do
			local use = fabs(dqCombo[i]) / limit
			rescale = rescale or use > 1
			table.insert(usage, use)
		end
		max_usage = max(unpack(usage))
		if rescale then
			for i = 1, #dqCombo do dqCombo[i] = dqCombo[i] / max_usage end
		end
		-- Apply the joint change
		qArm = qArm + dqCombo
		-- Check joint limit compliance
		for i, q in ipairs(qArm) do qArm[i] = min(max(qMin[i], q), qMax[i]) end
		-- Yield the progress
		qArmSensed, vwTargetNew, weightsNew, qArmFGuessNew = coroutine.yield(qArm, n)
		-- Smart adaptation
		vwTarget = vwTargetNew or vwTarget
		weights = weightsNew or weights
		qArmFGuess = qArmFGuessNew or qArmFGuess
		local fkArmSensed = forward(qArmSensed)
		if weightsNew and not qArmFGuessNew then
			qArmFGuess = self:find_shoulder(fkArmSensed, qArmSensed, weights) or qArmFGuess
		end
		-- If we are lagging badly, then there may be a collision
		local dqLag = qArm - qArmSensed
		local imax_lag, max_lag = 0, 0
		-- Use a higher tolerance here, since using position feedback
		for i, dq in ipairs(dqLag) do
			assert(fabs(dq-dqCombo[i])<5*DEG_TO_RAD, 'jacobian_preplan | Bad Lag')
		end
	until n > nStepsTimeout

	return qArm
end


-- Set the forward and inverse
local function set_chain(self, forward, inverse, jacobian)
	self.forward = assert(forward)
	self.inverse = assert(inverse)
	self.jacobian = jacobian
  return self
end

-- Set the iterator resolutions
local function set_limits(self, qMin, qMax, dqdt_limit)
	self.qMin = assert(qMin)
	self.qMax = assert(qMax)
	self.dqdt_limit = assert(dqdt_limit)
	self.qRange = qMax - qMin
  return self
end

-- Set the iterator resolutions
local function set_update_rate(self, hz)
	assert(type(hz)=='number', 'libArmPlan | Bad update rate for '..tostring(self.id))
	assert(type(self.dqdt_limit)=='table',
		'libArmPlan | Unknown dqdt_limit for '..tostring(self.id))
	self.hz = hz
	self.dt = 1/hz
	self.dq_limit = self.dqdt_limit / hz
  return self
end

local function set_shoulder_granularity(self, granularity)
	assert(type(granularity)=='number', 'Granularity not a number')
	local minShoulder, maxShoulder = self.qMin[3], self.qMax[3]
	local n = math.floor((maxShoulder - minShoulder) / granularity + 0.5)
	local shoulderAngles = {}
	for i=0,n do table.insert(shoulderAngles, minShoulder + i * granularity) end
	self.shoulderAngles = shoulderAngles
	return self
end

-- Still must set the forward and inverse kinematics
function libArmPlan.new_planner(id)
	local nq = 7
	local armOnes = vector.ones(nq)
	local armZeros = vector.zeros(nq)
	local obj = {
		id = id or 'Unknown',
		nq = nq,
		zeros = armZeros,
		ones = armOnes,
		halves = armOnes * 0.5,
		--
		shoulderAngles = nil,
		find_shoulder = find_shoulder,
		--
		set_chain = set_chain,
		set_limits = set_limits,
		set_update_rate = set_update_rate,
		set_shoulder_granularity = set_shoulder_granularity,
	}
	return obj
end

return libArmPlan
