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
local EPSILON = 1e-2 * DEG_TO_RAD

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

-- Use the Jacobian
local speed_eps = 0.1 * 0.1
local c, p = 2, 10
local torch = require'torch'
local function get_delta_qwaistarm(self, vwTarget, qArm, qWaist)
	-- Penalty for joint limits
	local qMin, qMax, qRange =
		{unpack(self.qMin)}, {unpack(self.qMax)}, {unpack(self.qRange)}

	assert(type(qArm)=='table', 'get_delta_qwaistarm | Bad qArm')
	assert(type(vwTarget)=='table', 'get_delta_qwaistarm | Bad vwTarget')

	local qWaistArm = {unpack(qArm)}
	if qWaist then
		table.insert(qWaistArm, 1, qWaist[1])
		table.insert(qMin, 1, -math.pi)
		table.insert(qMax, 1, math.pi)
		table.insert(qRange, 1, 2*math.pi)
	end

	local l = {}
	for i, q in ipairs(qWaistArm) do
    l[i]= speed_eps + c * ((2*q - qMin[i] - qMax[i])/qRange[i]) ^ p
  end
	-- Calculate the pseudo inverse
	--print('self.jacobian', qArm, qWaist)
	local J = torch.Tensor(self.jacobian(qArm, qWaist))
	local JT = J:t():clone()
	local lambda = torch.Tensor(l)
	local invInner = torch.inverse(torch.diag(lambda):addmm(JT, J))
	--print('invInner', invInner)
	local Jpseudoinv = torch.mm(invInner, JT)
	--print('Jpseudoinv', Jpseudoinv)
	local dqArm = torch.mv(Jpseudoinv, torch.Tensor(vwTarget))
	local null = torch.eye(#l) - Jpseudoinv * J
	return dqArm, null
end

local function get_distance(self, trGoal, qArm, qWaist)
	-- Grab our relative transform from here to the goal
	local fkArm = self.forward(qArm, qWaist)
	local invArm = T.inv(fkArm)
	local here = invArm * trGoal
	-- Determine the position and angular velocity target
	local dp = T.position(here)
	local drpy = T.to_rpy(here)
	local components = {vnorm(dp), vnorm(drpy)}
	return dp, drpy, components
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


-- Weights: cusage, cdiff, ctight, cshoulder, cwrist
local defaultWeights = {0, 0, 0, 0, 2}
--
local function valid_cost(iq, minArm, maxArm)
	for i, q in ipairs(iq) do
		if q<minArm[i] or q>maxArm[i] then return INFINITY end
	end
	return 0
end
local IK_POS_ERROR_THRESH = 0.03
-- TODO: Fix the find_shoulder api
local function find_shoulder(self, tr, qArm, weights, qWaist)
	weights = weights or defaultWeights
	-- Form the inverses

	local iqArms = {}
	for i, q in ipairs(self.shoulderAngles) do
		local iq = self.inverse(tr, qArm, q, 0, qWaist)
		local du = sanitize0(iq, qArm)
		tinsert(iqArms, iq)
	end
	-- Form the FKs
	local fks = {}
	for ic, iq in ipairs(iqArms) do
		fks[ic] = self.forward(iq, qWaist)
	end
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
	local ctight, wtight = {}, weights[3] or 0
	-- The margin from zero degrees away from the body
	local margin, ppi = 5*DEG_TO_RAD, math.pi
	for _, iq in ipairs(iqArms) do tinsert(ctight, fabs((iq[2]-margin))/ppi) end
	-- Usage cost (Worst Percentage)
	----[[
	local cusage, dRelative = {}
	for _, iq in ipairs(iqArms) do
		dRelative = ((iq - minArm) / rangeArm) - halfway
		-- Don't use the infinite yaw ones.  Treated later
		--tremove(dRelative, 7)
		--tremove(dRelative, 5)
		-- Don't use a cost based on the shoulderAngle
		tremove(dRelative, 3)
		tinsert(cusage, max(fabs(min(unpack(dRelative))), fabs(max(unpack(dRelative)))))
	end
	--]]
	-- Away from zero
	--[[
	local cusage, dRelative = {}
	for _, iq in ipairs(iqArms) do
		tinsert(cusage, max(fabs(min(unpack(iq))), fabs(max(unpack(iq)))))
	end
	--]]
	local cshoulder, wshoulder = {}, weights[4] or 0
	for _, iq in ipairs(iqArms) do
		tinsert(cshoulder, fabs(iq[3] - qArm[3]))
	end
	local cwrist, wwrist = {}, weights[5] or 2
	for _, iq in ipairs(iqArms) do
		tinsert(cwrist, fabs(iq[5]) + fabs(iq[7]))
	end


	-- Combined cost
	-- TODO: Tune the weights on a per-task basis (some tight, but not door)
	local cost = {}
	for ic, valid in ipairs(cvalid) do
		tinsert(cost, valid + cfk[ic]
			+ weights[1]*cusage[ic]
			+ weights[2]*cdiff[ic]
			+ wtight*ctight[ic]
			+ wshoulder*cshoulder[ic]
		)
	end
	-- Find the smallest cost
	local ibest, cbest = 0, INFINITY
	for i, c in ipairs(cost) do if c<cbest then cbest = c; ibest = i end end
	-- Return the least cost arms
	return iqArms[ibest], fks[ibest]
end

function libArmPlan.joint_preplan(self, plan)
	assert(type(plan)=='table', 'joint_preplan | Bad plan')
	local timeout = assert(plan.timeout, 'joint_preplan | No timeout')
	local qArm0 = assert(plan.qArm0, 'joint_preplan | Need initial arm')
	local qWaist0 = assert(plan.qWaist0, 'joint_preplan | Need initial waist')
	assert(plan.tr or plan.q, 'jacobian_preplan | Need tr or q')

	local weights = plan.weights
	local qArmF = plan.q
	if plan.tr then
		qArmF = self:find_shoulder(plan.tr, qArm0, weights)
	end
	assert(type(qArmF)=='table', 'joint_preplan | No target shoulder solution')
	local qMin, qMax = self.qMin, self.qMax
	-- Check joint limit compliance
	for i, q in ipairs(qArmF) do
		if qMin[i]~=-180*DEG_TO_RAD or qMax[i]~=180*DEG_TO_RAD then
			----[[
			assert(q+EPSILON>=qMin[i],
				string.format('joint_preplan | Below qMax[%d] %g < %g', i, q, qMin[i]))
			assert(q-EPSILON<=qMax[i],
				string.format('joint_preplan | Above qMin[%d] %g > %g', i, q, qMax[i]))
			--]]
			--qArmF[i] = min(max(qMin[i], q), qMax[i])
		end
	end
	-- Set the timeout
	local hz, dt = self.hz, self.dt
	local dq_limit = self.dq_limit
	local nStepsTimeout = timeout * hz
	-- If given a duration, then check speed limit compliance
	local dqAverage
	local duration = plan.duration
	if type(duration)=='number' then
		if Config.debug.armplan then
			print('joint_preplan | Using duration')
		end
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
	local qArm = vector.new(qArm0)
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
				for i, qF in ipairs(dqArmF) do dqArmF[i] = qF / max_usage2 end
			end
			dqUsed = dqArmF
		end
		-- Apply the joint change
		qArm = qArm + dqUsed
		table.insert(path, qArm)
		if (not dqAverage) and (dist < 0.5*DEG_TO_RAD) then break end
	until n > nStepsTimeout

	if Config.debug.armplan then
	  print('joint_preplan | Steps:', n)
	end
	assert(dqAverage or (n <= nStepsTimeout),
		'joint_preplan | Timeout: '..nStepsTimeout)

	-- Start the sensing
	local qArmSensed, qWaistSensed = coroutine.yield()
	qArmSensed = qArmSensedNew or qArm0
	qWaistSensed = qWaistSensedNew or qWaist0
	-- Progress is different, now, since in joint space
	for i, qArmPlanned in ipairs(path) do
		local qArmSensedNew, qWaistSensedNew = coroutine.yield(qArmPlanned)
		qArmSensed = qArmSensedNew or qArmSensed
		qWaistSensed = qWaistSensedNew or qWaistSensed
	end
	return qArmF
end

function libArmPlan.joint_waist_preplan(self, plan)
	assert(type(plan)=='table', 'joint_waist_preplan | Bad plan')
	local timeout = assert(plan.timeout, 'joint_waist_preplan | No timeout')
	local qArm0 = assert(plan.qArm0, 'joint_waist_preplan | Need initial arm')
	local qWaist0 = assert(plan.qWaist0, 'joint_waist_preplan | Need initial waist')
	assert(plan.tr or plan.q, 'joint_waist_preplan | Need tr or q')
	local qArmFGuess = plan.qArmGuess or qArm0
	local qWaistF = plan.qWaistGuess or qWaist0
	local qArmF = plan.q
	if plan.tr then
		qArmF = self:find_shoulder(plan.tr, qArmFGuess, weights, qWaistF)
	end
	assert(type(qArmF)=='table', 'joint_waist_preplan | No target shoulder solution')
	local qWaistArmF = vector.new({qWaistF[1], unpack(qArmF)})
	local qWaistArm0 = vector.new({qWaist0[1], unpack(qArm0)})
	-- Set the soft requirements
	local weights = plan.weights
	local hz, dt = self.hz, self.dt
	local nStepsTimeout = math.ceil(timeout * hz)
	local qMin = {-math.pi, unpack(self.qMin)}
	local qMax = {math.pi, unpack(self.qMax)}
	local dq_limit = {30*DEG_TO_RAD, unpack(self.dq_limit)}
	-- Check *soft* joint limit compliance
	for i, q in ipairs(qWaistArmF) do
		if qMin[i]~=-180*DEG_TO_RAD or qMax[i]~=180*DEG_TO_RAD then
			----[[
			assert(q+EPSILON>=qMin[i],
				string.format('joint_preplan | Below qMin[%d] %g < %g', i, q, qMin[i]))
			assert(q-EPSILON<=qMax[i],
				string.format('joint_preplan | Above qMax[%d] %g > %g', i, q, qMax[i]))
			--]]
			--qWaistArmF[i] = min(max(qMin[i], q), qMax[i])
		end
	end
	-- If given a duration, then check speed limit compliance
	local dqAverage
	local duration = plan.duration
	if type(duration)=='number' then
		if Config.debug.armplan then
		  print('joint_waist_preplan | Using duration')
		end
		assert(timeout>=duration,
			string.format('joint_waist_preplan | Timeout %g < Duration %g', timeout, duration))
		local dqTotal = qWaistArmF - qWaistArm0
		local dqdtAverage = dqTotal / duration
		dqAverage = dqdtAverage * dt
		for i, lim in ipairs(dq_limit) do
			assert(fabs(dqAverage[i]) <= lim,
				string.format("joint_waist_preplan | dq[%d] |%g| > %g", i, dqAverage[i], lim))
		end
		nStepsTimeout = duration * hz
	end

	local path = {}
	local qWaistArm = qWaistArm0
	n = 0
	repeat
		n = n + 1
		local dqWaistArmF = qWaistArmF - qWaistArm
		local dist = vnorm(dqWaistArmF)
		local dqUsed
		if dqAverage then
			dqUsed = dqAverage
		else
			-- Check the speed limit usage
			local usage, rescale = {}, false
			for i, limit in ipairs(dq_limit) do
				-- half speed?
				local use = fabs(dqWaistArmF[i]) / limit
				rescale = rescale or use > 1
				table.insert(usage, use)
			end
			if rescale then
				local max_usage2 = max(unpack(usage))
				for i, qF in ipairs(dqWaistArmF) do dqWaistArmF[i] = qF / max_usage2 end
			end
			dqUsed = dqWaistArmF
		end
		-- Apply the joint change
		qWaistArm = qWaistArm + dqUsed
		table.insert(path, qWaistArm)
		if (not dqAverage) and (dist < 0.5*DEG_TO_RAD) then break end
	until n > nStepsTimeout
	if Config.debug.armplan then
		print('joint_waist_preplan | Steps:', n)
	end
	assert(dqAverage or (n <= nStepsTimeout),
		'joint_waist_preplan | Timeout: '..nStepsTimeout)

	-- Start the sensing
	local qArmSensed, qWaistSensed = coroutine.yield()
	qArmSensed = qArmSensedNew or qArm0
	qWaistSensed = qWaistSensedNew or qWaist0
	-- Now replay
	for i, qWaistArmPlanned in ipairs(path) do
		local qArmSensedNew, qWaistSensedNew = coroutine.yield(
			{unpack(qWaistArmPlanned,2,#qWaistArmPlanned)},
			{qWaistArmPlanned[1], 0}
		)
		qArmSensed = qArmSensedNew or qArmSensed
		qWaistSensed = qWaistSensedNew or qWaistSensed
	end

	return {unpack(qWaistArmF,2,#qWaistArmF)}, {qWaistArmF[1], 0}
end

-- Plan a direct path using a straight line via Jacobian
-- res_pos: resolution in meters
-- res_ang: resolution in radians
function libArmPlan.jacobian_preplan(self, plan)
	assert(type(plan)=='table', 'jacobian_preplan | Bad plan')
	local qArm0 = assert(plan.qArm0, 'jacobian_preplan | Need initial arm')
	local qWaist0 = assert(plan.qWaist0, 'jacobian_preplan | Need initial waist')
	assert(plan.tr or plan.q, 'jacobian_preplan | Need tr or q')
	local trGoal = plan.tr or self.forward(plan.q)
	local timeout = assert(plan.timeout, 'jacobian_preplan | No timeout')
	local weights = plan.weights
	-- Find a guess of the final arm position
	local qWaistFGuess = vector.new(plan.qWaistGuess or qWaist0)
	local qArmGuess = plan.qArmGuess
	local qArmFGuess = qArmGuess or self:find_shoulder(trGoal, qArm0, weights, qWaistFGuess)
	vector.new(qArmFGuess)
	if Config.debug.armplan then
		if not qArmFGuess then print('jacobian_preplan | No guess found for the final!') end
	end

	local hz, dt = self.hz, self.dt
	local dq_limit = self.dq_limit
	local qMin, qMax = self.qMin, self.qMax
	local nStepsTimeout = math.ceil(timeout * hz)

	local t0 = unix.time()
	local qArm = vector.new(qArm0)
	local dp, drpy, dist_components =
		get_distance(self, trGoal, qArm, qWaist0)
	local done = false
	local n = 0
	local max_usage
	local path = {}
	repeat
		n = n + 1
		local vwTarget = {unpack(dp)}
		vwTarget[4], vwTarget[5], vwTarget[6] = unpack(drpy)
		-- Grab the joint velocities needed to accomplish the se(3) velocities
		local dqdtArm, nullspace = get_delta_qwaistarm(self, vwTarget, qArm)
		local dqdtCombo
		-- Grab the null space velocities toward our guessed configuration
		if qArmFGuess then 
			local dqdtNull = nullspace * torch.Tensor(qArm - qArmFGuess)
			dqdtCombo = dqdtArm - dqdtNull
		else
			dqdtCombo = dqdtArm
		end
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
		local qArmOld = qArm
		qArm = qArmOld + dqCombo
		-- Check joint limit compliance
		for i, q in ipairs(qArm) do
			if qMin[i]~=-180*DEG_TO_RAD or qMax[i]~=180*DEG_TO_RAD then
				qArm[i] = min(max(qMin[i], q), qMax[i])
			end
		end
		dp, drpy, dist_components = get_distance(self, trGoal, qArm, qWaist0)
		--print('dist_components', unpack(dist_components))

		--sanitize0(qArm, qArmOld)

		table.insert(path, qArm)

		-- Check if we are close enough
		if dist_components[1] < 0.01 and dist_components[2] < 2*RAD_TO_DEG then
			break
		end
	until n > nStepsTimeout
	local t1 = unix.time()

	-- Goto the final arm position as quickly as possible
	-- NOTE: We assume the find_shoulder yields a valid final configuration
	-- Use the last known max_usage to finalize
	if Config.debug.armplan then
	  print('jacobian_preplan '..self.id, n, 'steps planned: ', (t1-t0)..'s')
		print('jacobian_preplan | max_usage final', max_usage)
	end

	-- Goto the final
	local qArmF = qArm
	if(n > nStepsTimeout) then
		if Config.debug.armplan then
			print('jacobian_preplan | Timeout')
		end
	else
		qArmF = self:find_shoulder(trGoal, qArm, {0,1,0}, qWaist0) or qArm
	end

	n = 0
	nStepsTimeout = 5 * hz -- 3 second timeout to finish
	repeat
		n = n + 1
		local dqArmF = qArmF - qArm
		local dist = vnorm(dqArmF)
		if dist < 0.5*DEG_TO_RAD then break end
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
		table.insert(path, qArm)
	until n > nStepsTimeout
	if Config.debug.armplan then
		print('jacobian_preplan | Final steps', n)
		if(n > nStepsTimeout) then print('jacobian_preplan | Final timeout') end
	end

	-- Play the plan
	local qArmSensed, qWaistSensed = coroutine.yield()
	qArmSensed = qArmSensed or qArm0
	qWaistSensed = qWaistSensed or qWaist0
	for i, qArmPlanned in ipairs(path) do
		local qArmSensedNew, qWaistSensedNew = coroutine.yield(qArmPlanned)
		qArmSensed = qArmSensedNew or qArmSensed
		qWaistSensed = qWaistSensedNew or qWaistSensed
	end

	return qArmF
end

function libArmPlan.jacobian_waist_preplan(self, plan)
	assert(type(plan)=='table', 'jacobian_waist_preplan | Bad plan')
	local qArm0 = assert(plan.qArm0, 'jacobian_waist_preplan | Need initial arm')
	local qWaist0 = assert(plan.qWaist0, 'jacobian_waist_preplan | Need initial waist')
	assert(plan.tr or plan.q, 'jacobian_waist_preplan | Need tr or q')
	local trGoal = plan.tr or self.forward(plan.q)
	local timeout = assert(plan.timeout, 'jacobian_waist_preplan | No timeout')
	local weights = plan.weights
	-- Find a guess of the final arm position (With the given waist)
	local qWaistFGuess = plan.qWaistGuess or qWaist0
	local qArmFGuess = plan.qArmGuess or qArm0
	local qWaistArmFGuess = self:find_shoulder(trGoal, qArmFGuess, weights, qWaistFGuess)
	assert(qWaistArmFGuess, 'jacobian_waist_preplan | No guess found for the final!')
	vector.new(qWaistArmFGuess)
	table.insert(qWaistArmFGuess, 1, qWaistFGuess[1])
	local hz, dt = self.hz, self.dt
	local nStepsTimeout = math.ceil(timeout * hz)
	local qMin = {-math.pi, unpack(self.qMin)}
	local qMax = {math.pi, unpack(self.qMax)}
	local dq_limit = {30*DEG_TO_RAD, unpack(self.dq_limit)}
	-- Starting qWaistArm
	local qWaistArm = vector.new{qWaist0[1], unpack(qArm0)}
	local dp, drpy, dist_components =
		get_distance(self, trGoal, qArm0, qWaist0)
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
		local dqdtArm, nullspace = get_delta_qwaistarm(
			self,
			vwTarget,
			{unpack(qWaistArm,2,#qWaistArm)},
			{qWaistArm[1], 0}
		)
		-- Grab the null space velocities toward our guessed configuration
		local dqdtNull = nullspace * torch.Tensor(qWaistArm - qWaistArmFGuess)
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
		local qWaistArmOld = qWaistArm
		qWaistArm = qWaistArmOld + dqCombo
		-- Check joint limit compliance
		for i, q in ipairs(qWaistArm) do
			if qMin[i]~=-180*DEG_TO_RAD or qMax[i]~=180*DEG_TO_RAD then
				qWaistArm[i] = min(max(qMin[i], q), qMax[i])
			end
		end
		-- Yield the progress
		dp, drpy, dist_components = get_distance(self, trGoal,
		{unpack(qWaistArm,2,#qWaistArm)},
		{qWaistArm[1],0}
		)
		table.insert(path, qWaistArm)
		-- Check if we are close enough
		if dist_components[1] < 0.01 and dist_components[2] < 2*RAD_TO_DEG then
			break
		end
	until n > nStepsTimeout
	local t1 = unix.time()
	if Config.debug.armplan then
		print(n, 'jacobian_waist_preplan steps planned in: ', t1-t0)
	end
	assert(n <= nStepsTimeout, 'jacobian_waist_preplan | Timeout')

	local qWaistArmF = self:find_shoulder(
		trGoal, {unpack(qWaistArm,2,#qWaistArm)}, {0,1,0}, {qWaistArm[1], 0})
	assert(qWaistArmF, 'jacobian_preplan | No final shoulder solution')
	table.insert(qWaistArmF, 1, qWaistArm[1])

	-- Goto the final arm position as quickly as possible
	-- NOTE: We assume the find_shoulder yields a valid final configuration
	-- Use the last known max_usage to finalize
	if Config.debug.armplan then
		print('jacobian_waist_preplan | max_usage final', max_usage)
	end
	n = 0
	nStepsTimeout = 5 * hz -- 3 second timeout to finish
	repeat
		n = n + 1
		local dqWaistArmF = qWaistArmF - qWaistArm
		local dist = vnorm(dqWaistArmF)
		-- Check the speed limit usage
		local usage, rescale = {}, false
		for i, limit in ipairs(dq_limit) do
			-- half speed?
			local use = fabs(dqWaistArmF[i]) / (limit*max_usage)
			rescale = rescale or use > 1
			table.insert(usage, use)
		end
		if rescale then
			local max_usage2 = max(unpack(usage))
			--print('Rescaling 2!', max_usage2)
			for i, qF in ipairs(dqWaistArmF) do
				dqWaistArmF[i] = qF / max_usage2
			end
		end
		-- Apply the joint change
		qWaistArm = qWaistArm + dqWaistArmF
		table.insert(path, qWaistArm)
		if dist < 0.5*DEG_TO_RAD then break end
	until n > nStepsTimeout
	if Config.debug.armplan then
		print('jacobian_waist_preplan | Final Steps:', n)
	end
	assert(n <= nStepsTimeout, 'jacobian_waist_preplan | Final timeout')

	-- This gives our guessed final configuration.
	-- This may not be at all correct, since we are jacobian in waist, too, and do not search over waist angles
	local qArmSensed, qWaistSensed = coroutine.yield()
	qArmSensed = qArmSensed or qArm0
	qWaistSensed = qWaistSensed or qWaist0
	-- Now replay
	for i, qWaistArmPlanned in ipairs(path) do
		local qArmSensedNew, qWaistSensedNew = coroutine.yield(
			{unpack(qWaistArmPlanned,2,#qWaistArmPlanned)},
			{qWaistArmPlanned[1], 0}
		)
		qArmSensed = qArmSensedNew or qArmSensed
		qWaistSensed = qWaistSensedNew or qWaistSensed
	end

	return {unpack(qWaistArmF,2,#qWaistArmF)}, {qWaistArmF[1], 0}
end

-- resume with: qArmSensed, vwTargetNew, weightsNew
function libArmPlan.jacobian_velocity(self, plan)
	assert(type(plan)=='table',
		'jacobian_velocity | Bad plan')
	local vwTarget = plan.vw
	assert(type(vwTarget)=='table' and #vwTarget==6,
		'jacobian_velocity | Bad vw')
	local qArm0 = assert(plan.qArm0, 'Need initial arm')
	local qWaist0 = assert(plan.qWaist0, 'Need initial waist')

	local weights = plan.weights
	local hz, dt = self.hz, self.dt
	local dq_limit = self.dq_limit
	local qMin, qMax = self.qMin, self.qMax
	local forward = self.forward

	-- Find a guess of the final arm position
	local qArmFGuess = plan.qArmGuess or qArm0
	local qWaistFGuess = plan.qWaistGuess or qWaist0

	local qArm = qArm0

	local qArmSensed, qWaistSensed, vwTargetNew, weightsNew, qArmFGuessNew =
		coroutine.yield()
	vwTarget = vwTargetNew or vwTarget
	weights = weightsNew or weights
	qArmFGuess = qArmFGuessNew or qArmFGuess

	local done = false
	local n = 0
	local max_usage
	repeat
		n = n + 1
		-- Grab the joint velocities needed to accomplish the se(3) velocities
		local dqdtArm, nullspace =
			get_delta_qwaistarm(self, vwTarget, qArmSensed)
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

		local qArmOld = qArm
		qArm = qArmOld + dqCombo

		-- Check joint limit compliance
		for i, q in ipairs(qArm) do
			if qMin[i]~=-180*DEG_TO_RAD or qMax[i]~=180*DEG_TO_RAD then
				qArm[i] = min(max(qMin[i], q), qMax[i])
			end
		end
		-- Yield the progress
		--print('Next yield',qArmOld, dqCombo, qArm)
		qArmSensed, qWaistSensed, vwTargetNew, weightsNew, qArmFGuessNew =
			coroutine.yield(qArm)
		-- Smart adaptation
		vwTarget = vwTargetNew or vwTarget
		weights = weightsNew or weights
		qArmFGuess = qArmFGuessNew or qArmFGuess
		--print('qArmFGuessNew', qArmFGuessNew, qArmFGuess)
		if weightsNew then
			local fkArmSensed = forward(qArmSensed)
			local qArmFGuessNew = self:find_shoulder(fkArmSensed, qArmSensed, weights)
			if not qArmFGuessNew then
				print('jacobian_velocity | Bad velocity guess')
			else
				qArmFGuess = qArmFGuessNew
			end
			--assert(qArmFGuess, 'Bad guess!')
			----or qArmFGuess
			--print('qArmFGuess', vector.new(qArmFGuess))
			--print('qArmSensed', qArmSensed)
			--print(vector.norm(qArmFGuess-qArmSensed)*RAD_TO_DEG, 'diff', (qArmFGuess-qArmSensed)*RAD_TO_DEG)
			if(math.abs(qArmFGuess[3]-qArmSensed[3])>10*DEG_TO_RAD) then
				print('jacobian_velocity | wait!')
				vwTarget = {0,0,0, 0,0,0}
			end
		end
		-- If we are lagging badly, then there may be a collision
		local dqLag = qArm - qArmSensed
		local imax_lag, max_lag = 0, 0
		-- Use a higher tolerance here, since using position feedback
		for i, dq in ipairs(dqLag) do
			--assert(fabs(dq-dqCombo[i])<5*DEG_TO_RAD, 'jacobian_preplan | Bad Lag')
		end
	until vwTargetNew==false

	return qArm
end

-- resume with: qArmSensed, vwTargetNew, weightsNew
function libArmPlan.jacobian_waist_velocity(self, plan)
	assert(type(plan)=='table',
		'jacobian_velocity | Bad plan')
	local vwTarget = plan.vw
	assert(type(vwTarget)=='table' and #vwTarget==6,
		'jacobian_velocity | Bad vw')
	local qArm0 = assert(plan.qArm0, 'Need initial arm')
	local qWaist0 = assert(plan.qWaist0, 'Need initial waist')

	local forward = self.forward
	local weights = plan.weights
	local hz, dt = self.hz, self.dt

	local qMin, qMax =
		{-math.pi,unpack(self.qMin)}, {math.pi,unpack(self.qMax)}
	local dq_limit = {30*DEG_TO_RAD, unpack(self.dq_limit)}

	-- Track here
	local qWaistArm = vector.new{qWaist0[1], unpack(qArm0)}

	-- Find a guess of the final arm position
	local qArmFGuess = plan.qArmGuess or qArm0
	local qWaistFGuess = plan.qWaistGuess or qWaist0
	local qWaistArmFGuess = {qWaistFGuess[1], unpack(qArmFGuess)}


	local qArmSensed, qWaistSensed, vwTargetNew, weightsNew, qArmFGuessNew, qWaistFGuessNew =
		coroutine.yield()
	vwTarget = vwTargetNew or vwTarget
	weights = weightsNew or weights
	qArmFGuess = qArmFGuessNew or qArmFGuess

	local done = false
	local n = 0
	local max_usage
	repeat
		n = n + 1

		-- Grab the joint velocities needed to accomplish the se(3) velocities
		local dqdtWaistArm, nullspace = get_delta_qwaistarm(
			self,
			vwTarget,
			qArmSensed,
			qWaistSensed
		)
		-- Grab the null space velocities toward our guessed configuration
		local dqdtNull = nullspace * torch.Tensor(qWaistArm - qWaistArmFGuess)
		-- Linear combination of the two
		local dqdtCombo = dqdtWaistArm - dqdtNull
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

		local qWaistArmOld = qWaistArm
		qWaistArm = qWaistArmOld + dqCombo

		-- Check joint limit compliance
		for i, q in ipairs(qWaistArm) do
			if qMin[i]~=-180*DEG_TO_RAD or qMax[i]~=180*DEG_TO_RAD then
				qWaistArm[i] = min(max(qMin[i], q), qMax[i])
			end
		end
		-- Yield the progress
		--print('Next yield',qArmOld, dqCombo, qArm)
		qArmSensed, qWaistSensed, vwTargetNew, weightsNew, qArmFGuessNew =
			coroutine.yield({unpack(qWaistArm,2,#qWaistArm)}, {qWaistArm[1], 0})
		-- Smart adaptation
		vwTarget = vwTargetNew or vwTarget
		weights = weightsNew or weights
		qArmFGuess = qArmFGuessNew or qArmFGuess
		qWaistFGuess = qWaistFGuessNew or qWaistFGuess
		--print('qArmFGuessNew', qArmFGuessNew, qArmFGuess)
		if weightsNew then
			local fkArmSensed = forward(qArmSensed, qWaistSensed)
			local qArmFGuessNew =
				self:find_shoulder(fkArmSensed, qArmSensed, weights, qWaistSensed)
			if qArmFGuessNew then
				qArmFGuess = qArmFGuessNew
			else
				print('jacobian_velocity | Bad velocity guess')
			end
			-- Search space yields a new independent variable result
			-- If too far away, then adjust in null space only
			-- TODO: This should be on all updates
		end
		if qArmFGuessNew or qWaistFGuessNew or weightsNew then
			qWaistArmFGuess = vector.new{qWaistFGuess[1], unpack(qArmFGuess)}
			--print('New qWaistArmFGuess', vector.new(qWaistArmFGuess)*RAD_TO_DEG)
			--print('diff',(qWaistArmFGuess-qWaistArm)*RAD_TO_DEG)
		end

		-- Make sure we can always move
		if(math.abs(qWaistArmFGuess[4]-qWaistArm[4])>6*DEG_TO_RAD) then
			--print('jacobian_velocity | wait!')
			vwTarget = {0,0,0, 0,0,0}
		end

		-- If we are lagging badly, then there may be a collision
		--[[
		local dqLag = qArm - qArmSensed
		local imax_lag, max_lag = 0, 0
		-- Use a higher tolerance here, since using position feedback
		for i, dq in ipairs(dqLag) do
			--assert(fabs(dq-dqCombo[i])<5*DEG_TO_RAD, 'jacobian_preplan | Bad Lag')
		end
		--]]
	until vwTargetNew==false

	return {unpack(qWaistArm,2,#qWaistArm)}, {qWaistArm[1], 0}
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
