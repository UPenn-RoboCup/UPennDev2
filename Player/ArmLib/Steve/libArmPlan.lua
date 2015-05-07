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
	return iqArms[ibest], fks[ibest]
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
	local hz, dt = self.hz, self.dt
	local nSteps = math.ceil(duration * hz)
	-- Set the path
	local qArm = qArm0
	-- Yield the joint values and % complete
	coroutine.yield()
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
function libArmPlan.jacobian_preplan(self, trGoal, qArm0, shoulder_weights, timeout)
	local hz, dt = self.hz, self.dt
	local dq_limit = self.dq_limit
	local qMin, qMax = self.qMin, self.qMax
	-- 10 second default timeout
	timeout = math.ceil(timeout or 10)
	-- What is the weight of the null movement?
	local alpha_n = 0.25
	-- Find a guess of the final arm position
	local qArmFGuess = self:find_shoulder(trGoal, qArm0, shoulder_weights)
	--assert(qArmFGuess, 'jacobian_preplan | No guess shoulder solution')
	qArmFGuess = qArmFGuess or qArm0
	local qArm = qArm0
	--print('get_distance')
	local dp, drpy, dist_components = get_distance(self, qArm, trGoal)
	local dp0, drpy0, dist_components0 = dp, drpy, dist_components
	--print('dist_components', unpack(dist_components))
	coroutine.yield(qArmFGuess, dist_components)

	local done = false
	local n = 0
	local max_usage
	repeat
		n = n + 1
		local vwTarget = {unpack(dp)}
		vwTarget[4], vwTarget[5], vwTarget[6] = unpack(drpy)
		-- Grab the joint velocities needed to accomplish the se(3) velocities
		local dqdtArm, nullspace = get_delta_qarm(self, vwTarget, qArm)
		-- Grab the null space velocities toward our guessed configuration
		local dqdtNull = nullspace * torch.Tensor(qArm - qArmFGuess)
		-- Linear combination of the two
		local dqdtCombo = dqdtArm:mul(1-alpha_n) - dqdtNull:mul(alpha_n)
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
		coroutine.yield(qArm, dist_components)
		-- Check if we are close enough
		if dist_components[1] < 0.02 and dist_components[2] < 2*RAD_TO_DEG then
			break
		end
	until n > timeout * hz
	print(n, 'jacobian steps')
	assert(n <= 1e3, 'Jacobian stack is stuck')
	local qArmF = self:find_shoulder(trGoal, qArm, {0,1,0})
	--assert(qArmF, 'jacobian_preplan | No final shoulder solution')
	qArmF = qArmF or qArm

	-- Goto the final arm position as quickly as possible
	-- NOTE: We assume the find_shoulder yields a valid final configuration
	-- Use the last known max_usage to finalize
	print('max_usage final', max_usage)
	n = 0
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
		coroutine.yield(qArm, dqArmF)
		--print('final dist', dist*RAD_TO_DEG)
		if dist < 0.5*DEG_TO_RAD then break end
	until n > 3*hz -- 3 second timeout to finish
	print(n, 'final steps')

	return qArmF
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
	print('granularity', granularity)
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
