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
local util = require'util'

-- Does not work for the infinite turn motors
local function sanitize(qPlanned, qNow)
	local qDiff = qPlanned - qNow
	local qDiffEffective = mod_angle(qDiff)
	if fabs(qDiffEffective) < fabs(qDiff) then
		return qNow + qDiffEffective
	else
		return qNow + qDiff
	end
end

local function sanitizeAll(iqArm, qArm0)
	local iqArm2 = {}
	for i, qNow in ipairs(qArm0) do
		if (i==5 or i==7) then
			-- TODO: Find the nearest
			iqArm2[i] = iqArm[i]
		else
			iqArm2[i] = sanitize(iqArm[i], qNow)
		end
	end
	return iqArm2
end

local function qDiff(iqArm, qArm0, qMin, qMax)
	local qD = {}
	for i, q0 in ipairs(qArm0) do
		qD[i] = (i==5 or i==7) and (iqArm[i] - q0) or sanitize(iqArm[i], q0)
	end
	return qD
end


-- Calculate the pseudo inverse
--print('self.jacobian', qArm, qWaist)
	--[[
	Thus, the damped least squares solution is equal to
	∆θ = (J^T * J + λ^2 * I)^−1 * J^T * e
	--]]
	--[[
	TODO:
	It is easy to show that (J T J + λ 2 I) −1 J T = J T (JJ T + λ 2 I) −1 . Thus,
	∆θ = J^T *(J * J^T + λ^2 * I)^−1 *e
	--]]
	--[[
	TODO:
	Additionally, (11) can be computed without needing to carry out the
matrix inversion, instead row operations can find f such that (JJ T +λ 2 I) f =
e and then J T f is the solution.
	--]]
	--[[
	-- TODO: Robot subtask performance with singularity robustness using optimal damped least-squares
	While (1) is not defined for λ = 0, (2) is as the
matrix (J*JT + λI) is invertible for λ = 0 provided J
has full row rank.
	--]]
local speed_eps = 0.1 * 0.1
local c, p = 2, 10
local torch = require'torch'
local mattorch = require'mattorch'
local function get_pseudo_jacobian_dls(self, J, qWaistArm)
	-- Penalty for joint limits
	local qMin = self.qMin
	local qMax = self.qMax
	local qRange = self.qRange
	if #qWaistArm~=#qRange then
		qMin = {-45*DEG_TO_RAD, unpack(self.qMin)}
		qMax = {45*DEG_TO_RAD, unpack(self.qMax)}
		qRange = {90*DEG_TO_RAD, unpack(self.qRange)}
	end
	local l = {}
	for i, q in ipairs(qWaistArm) do
    l[i]= speed_eps + c * ((2*q - qMin[i] - qMax[i])/qRange[i]) ^ p
  end
	local JT = J:t()
	local invInner = torch.inverse(
		torch.addmm(torch.diag(torch.Tensor(l)), JT, J)
	)
	local Jpseudoinv = torch.mm(invInner, JT)
	return Jpseudoinv
end

-- Use the Jacobian
local function get_nullspace(self, qArm, qWaist)

	assert(type(qArm)=='table', 'get_delta_qwaistarm | Bad qArm')

	local qWaistArm = qArm
	if qWaist then
		qWaistArm = {qWaist[1], unpack(qArm)}
	end

	local J = torch.Tensor(self.jacobian(qArm, qWaist))
	-- Straight Pseudo inverse
	--local Jpseudoinv = torch.mm(torch.inverse(JT*J), JT)
	-- Simplification for less degrees of freedom: easier to calculate
	--local Jpseudoinv = torch.mm(JT, torch.inverse(J * JT))
	-- Damped Least Squares Pseudo Inverse
	local Jpseudoinv = get_pseudo_jacobian_dls(self, J, qWaistArm)
	--local null = torch.eye(#qWaistArm) - Jpseudoinv * J
	local null = torch.addmm(
		torch.eye(#qWaistArm),
		-1,
		Jpseudoinv,
		J)

	return null, J, Jpseudoinv
end

local function get_distance(self, trGoal, qArm, qWaist)
	-- Grab our relative transform from here to the goal
	local fkArm = self.forward(qArm, qWaist)
	local invArm = T.inv(fkArm)
	local here = trGoal * invArm -- new good one

	-- Determine the position and angular velocity target
	local dp = T.position(here)
	local drpy = T.to_rpy(here)
	-- TODO: Add the rpy check for other rotation directions

	return dp, drpy
end

-- Similar to SJ's method for the margin
local function find_shoulder_sj(self, tr, qArm)
	local minArm, maxArm, rangeArm = self.qMin, self.qMax, self.qRange
	local iqArm, margin, qBest
	local dMin, dMax
	--
	local maxmargin, margin = -INFINITY
	for i, q in ipairs(self.shoulderAngles) do
		iqArm = self.inverse(tr, qArm, q)
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
local function valid_cost(iq, qMin, qMax)
	for i, q in ipairs(iq) do
		if i==5 or i==7 then --inf turn
		elseif q<qMin[i]-EPSILON or q>qMax[i]+EPSILON then return INFINITY end
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
		local iq2 = sanitizeAll(iq, qArm)
		tinsert(iqArms, vector.new(iq2))
	end
	-- Form the FKs
	local fks = {}
	for ic, iq in ipairs(iqArms) do
		fks[ic] = self.forward(iq, qWaist)
	end
	--
	local qMin, qMax = self.qMin, self.qMax
	local rangeArm, halfway = self.qRange, self.halves
	-- Cost for valid configurations
	local cvalid = {}
	for ic, iq in ipairs(iqArms) do tinsert(cvalid, valid_cost(iq, qMin, qMax) ) end
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
		tinsert(cdiff, vnorm(qDiff(iq, qArm, qMin, qMax)))
		--tinsert(cdiff, vnorm(iq - qArm))
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
		dRelative = ((iq - qMin) / rangeArm) - halfway
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
	local prefix = string.format('joint_preplan (%s) | ', self.id)
	assert(type(plan)=='table', prefix..'Bad plan')
	-- With a pre-existing path, first qArm is end of path
	local qArm0 = plan.qArm0
	if type(plan.qPath)=='table' then
		qArm0 = plan.qPath[#plan.qPath]
	end
	assert(qArm0, prefix..'Need initial arm')
	-- With a pre-existing path, first qWaist is end of path
	local qWaist0 = plan.qWaist0
	if type(plan.wPath)=='table' then
		qWaist0 = plan.wPath[#plan.wPath]
	end
	assert(qWaist0, prefix..'Need initial waist')
	--  Set the Final Goal for the arm
	local qArmF
	if type(plan.qGoal)=='table' then
		qArmF = plan.qGoal
	elseif type(plan.q)=='table' then
		qArmF = plan.q
	elseif plan.tr then
		local qArmFGuess = plan.qArmGuess or qArm0
		qArmF = self:find_shoulder(plan.tr, qArmFGuess, plan.weights, qWaist0)
		assert(type(qArmF)=='table', prefix..'No target shoulder solution')
	else
		error(prefix..'Need tr or q')
	end
	plan.qGoal = qArmF

	-- Set the limits and check compliance
	local qMin, qMax = self.qMin, self.qMax
	local dq_limit = self.dq_limit
	for i, q in ipairs(qArmF) do
		if i==5 or i==7 then
			-- No limit for infinite rotation :P
			--qArmF[i] = sanitize(qArmF[i], qArm0[i])
		else
			qArmF[i] = min(max(qMin[i], q), qMax[i])
		end
	end
	-- Set the timeout
	local hz, dt = self.hz, self.dt
	local qArm = vector.new(qArm0)
	-- Continue or start anew
	plan.qPath = plan.qPath or {}
	plan.nulls = plan.nulls or {}
	-- If given a duration, then check speed limit compliance

	if type(plan.duration)=='number' then
		if Config.debug.armplan then
			print(prefix..'Using duration:', plan.duration)
		end
		local dqTotal = qArmF - qArm0
		local dqdtAverage = dqTotal / plan.duration
		local dqAverage = dqdtAverage * dt
		local usage = {}
		for i, limit in ipairs(dq_limit) do
			if fabs(dqAverage[i]) > limit then
				print(string.format(prefix.."dq[%d] |%g| > %g", i, dqAverage[i], lim))
				--return qArm
			end
			table.insert(usage, fabs(dqAverage[i]) / limit)
		end
		local max_usage = max(unpack(usage))
		if max_usage>1 then
			for i, qF in ipairs(dqAverage) do
				dqAverage[i] = qF / max_usage
			end
		end
		-- Form the plan
		local nsteps = plan.duration * hz
		for i=1,nsteps do
			qArm = qArm + dqAverage
			table.insert(plan.qPath, qArm)
			local nullspace = get_nullspace(self, qArm)
			table.insert(plan.nulls, nullspace)
		end

		print(prefix..'Duration Steps:', #plan.qPath)
		return plan
	end
	-- Timeout based
	local timeout = assert(plan.timeout, prefix..'No timeout')
	local nStepsTimeout = timeout * hz
	local n = 0
	repeat
		local dqArmF = qArmF - qArm
		local dist = vnorm(dqArmF)
		if dist < 0.5*DEG_TO_RAD then
			break
		end
		-- Check the speed limit usage
		local usage = {}
		for i, limit in ipairs(dq_limit) do
			table.insert(usage, fabs(dqArmF[i]) / limit)
		end
		local max_usage = max(unpack(usage))
		if max_usage>1 then
			for i, qF in ipairs(dqArmF) do dqArmF[i] = qF / max_usage end
		end
		-- Apply the joint change
		qArm = qArm + dqArmF
		table.insert(plan.qPath, qArm)
		local nullspace = get_nullspace(self, qArm)
		table.insert(plan.nulls, nullspace)
		n = n + 1
	until n > nStepsTimeout

	if Config.debug.armplan then
	  print(prefix, n..' steps')
		--util.ptable(final_plan)
	end

	-- Finish
	if #plan.qPath > nStepsTimeout then
		if Config.debug.armplan then
			print(prefix..'Timeout: ', #plan.qPath)
		end
	end
	--return co_play(self, path)
	return plan
end

function libArmPlan.joint_waist_preplan(self, plan)
	local prefix = string.format('joint_waist_preplan (%s) | ', self.id)
	assert(type(plan)=='table', prefix..'Bad plan')
	local qArm0 = assert(plan.qArm0, prefix..'Need initial arm')
	local qWaist0 = assert(plan.qWaist0, prefix..'Need initial waist')
	-- If calling joint_waist, then should always have a final waist...
	local qWaistF = assert(plan.qWaistGuess, prefix..'Need final waist')
	local qArmF
	if type(plan.q)=='table' then
		qArmF = plan.q
	elseif plan.tr then
		local qArmFGuess = plan.qArmGuess or qArm0
		qArmF = self:find_shoulder(plan.tr, qArmFGuess, plan.weights, qWaistF)
		assert(type(qArmF)=='table', prefix..'No target shoulder solution')
	else
		error(prefix..'Need tr or q')
	end
	-- Form the waist/arm combo
	local qWaistArmF = {qWaistF[1], unpack(qArmF)}
	local qWaistArm0 = {qWaist0[1], unpack(qArm0)}
	-- Set the limits and check compliance
	local hz, dt = self.hz, self.dt
	local qMin = {-math.pi/3, unpack(self.qMin)}
	local qMax = {math.pi/3, unpack(self.qMax)}
	local dq_limit = {8*DEG_TO_RAD*dt, unpack(self.dq_limit)}

	-- Fix up the joint preplan
	-- TODO: this does not look right
	for i, q in ipairs(qWaistArmF) do
		if i==5 or i==7 then
			--qArmF[i] = sanitize1(qArmF[i], qArm0[i])
		else
			--[[
			assert(q+EPSILON>=qMin[i],
				string.format('%s Below qMin[%d] %g < %g', prefix, i, q, qMin[i]))
			assert(q-EPSILON<=qMax[i],
				string.format('%s Above qMax[%d] %g > %g', prefix, i, q, qMax[i]))
			--]]
			qWaistArmF[i] = min(max(qMin[i], q), qMax[i])
		end
	end
	-- Set the timeout
	local qWaistArm = vector.new(qWaistArm0)
	local path = {}
	-- If given a duration, then check speed limit compliance
	local duration = plan.duration
	if type(plan.duration)=='number' then
		if Config.debug.armplan then print(prefix..'Using duration:', plan.duration) end
		local dqTotal = qWaistArmF - qWaistArm0
		local dqdtAverage = dqTotal / plan.duration
		local dqAverage = dqdtAverage * dt
		for i, lim in ipairs(dq_limit) do
			assert(fabs(dqAverage[i]) <= lim,
				string.format("%s dq[%d] |%g| > %g", prefix, i, dqAverage[i], lim))
		end
		-- Form the plan
		local nsteps = plan.duration * hz
		for i=1,nsteps do
			qWaistArm = qWaistArm + dqAverage
			table.insert(path, qWaistArm)
		end
		print(prefix..'Steps:', #path)

		local qPath = {}
		local wPath = {}
		for i, qWaistArmPlanned in ipairs(path) do
			qPath[i] = {unpack(qWaistArmPlanned, 2, #qWaistArmPlanned)}
			wPath[i] = {qWaistArmPlanned[1], 0}
		end
		plan.qPath = qPath
		plan.wPath = wPath

		return co_play_waist(plan)
	end
	-- Timeout based
	local timeout = assert(plan.timeout, prefix..'No timeout')
	local nStepsTimeout = math.ceil(timeout * hz)
	repeat
		local dqWaistArmF = qWaistArmF - qWaistArm
		local dist = vnorm(dqWaistArmF)
		if dist < 0.5*DEG_TO_RAD then break end
		-- Check the speed limit usage
		local usage = {}
		for i, limit in ipairs(dq_limit) do
			table.insert(usage, fabs(dqWaistArmF[i]) / limit)
		end
		local max_usage = max(unpack(usage))
		if max_usage>1 then
			for i, qF in ipairs(dqWaistArmF) do dqWaistArmF[i] = qF / max_usage end
		end
		-- Apply the joint change
		qWaistArm = qWaistArm + dqWaistArmF
		table.insert(path, qWaistArm)
	until #path > nStepsTimeout
	-- Finish
	if Config.debug.armplan then
		print(prefix..'Steps:', #path)
		if #path > nStepsTimeout then print(prefix..'Timeout: ', #path) end
	end
	local qPath = {}
	local wPath = {}
	for i, qWaistArmPlanned in ipairs(path) do
		qPath[i] = {unpack(qWaistArmPlanned, 2, #qWaistArmPlanned)}
		wPath[i] = {qWaistArmPlanned[1], 0}
	end
	plan.qPath = qPath
	plan.wPath = wPath

	return co_play_waist(plan)
end

-- Plan via Jacobian for just the arm
function libArmPlan.jacobian_preplan(self, plan)
	local prefix = string.format('jacobian_preplan (%s) | ', self.id)
	assert(type(plan)=='table', prefix..'Bad plan')
	-- With a pre-existing path, first qArm is end of path
	local qArm0 = plan.qArm0
	if type(plan.qPath)=='table' then
		qArm0 = plan.qPath[#plan.qPath]
	end
	assert(qArm0, prefix..'Need initial arm')
	-- With a pre-existing path, first qWaist is end of path
	local qWaist0 = plan.qWaist0
	if type(plan.wPath)=='table' then
		qWaist0 = plan.wPath[#plan.wPath]
	end
	assert(qWaist0, prefix..'Need initial waist')
	-- Find a guess of the final arm configuration
	assert(plan.q or plan.tr, prefix..'Need tr or q')
	local trGoal
	local qArmFGuess
	if type(plan.q)=='table' then
		trGoal = self.forward(plan.q, qWaist0)
		plan.tr = trGoal
		qArmFGuess = plan.q
	elseif plan.tr then
		trGoal = plan.tr
		local weights = plan.weights
		qArmFGuess = plan.qArmGuess or self:find_shoulder(trGoal, qArm0, weights, qWaist0)
	end
	-- Use straight jacobian if no guess
	if qArmFGuess then
		plan.qArmFGuess = vector.new(qArmFGuess)
	else
		if Config.debug.armplan then
			print(prefix..'No guess found for the final!')
		end
	end
	-- Update a guess for the final waist
	local qWaistFGuess = plan.qWaistGuess or qWaist0
	-- Set the timing
	local timeout = assert(plan.timeout, prefix..'No timeout')
	local hz, dt = self.hz, self.dt
	local nStepsTimeout = math.ceil(timeout * hz)
	-- Grab our limits
	local dq_limit = self.dq_limit
	local qMin, qMax = self.qMin, self.qMax
	-- Initial position
	local qArm = vector.copy(qArm0)
	local dTF
	-- Null space variables
	local nullFactor = 0.3 --0.2
	local dqdtNull = torch.Tensor(#qArm)
	-- Continue the path
	plan.nulls = plan.nulls or {}
	plan.qPath = plan.qPath or {}
	-- Begin
	local t0 = unix.time()
	repeat
		-- Check if we are close enough
		local dp, drpy = get_distance(self, trGoal, qArm, qWaist0)
		dTF = vector.new{vnorm(dp), vnorm(drpy)}
		if dTF[1] < 0.01
			and dTF[2] < 2*DEG_TO_RAD
		then
			break
		end
		-- Form our desired velocity
		local vwTarget = torch.Tensor{
			dp[1], dp[2], dp[3],
			drpy[1], drpy[2], drpy[3],
		}
		-- Find the nullspace and Jacobian
		local nullspace, J, Jinv = get_nullspace(self, qArm)
		table.insert(plan.nulls, nullspace)
		-- Joint velocities to accomplish the se(3) velocities
		local dqdtArm = torch.mv(Jinv, vwTarget)
		local dqdtCombo
		if qArmFGuess then
			local dqNull = torch.Tensor(qArm - qArmFGuess)
			torch.mv(dqdtNull, nullspace, dqNull)
			dqdtCombo = dqdtArm - dqdtNull:mul(nullFactor)
		else
			dqdtCombo = dqdtArm
		end
		-- Respect the update rate, place as a lua table
		local dqCombo = vector.new(dqdtCombo:mul(dt))
		-- Check the speed limit usage
		local usage = {}
		for i, limit in ipairs(dq_limit) do
			table.insert(usage, fabs(dqCombo[i]) / limit)
		end
		local max_usage = max(unpack(usage))
		if max_usage > 1 then
			for i, dq in ipairs(dqCombo) do
				dqCombo[i] = dq / max_usage
			end
		end
		-- Apply the joint change
		local qOld = qArm
		qArm = qArm + dqCombo
		-- Check joint limit compliance
		for i, q in ipairs(qArm) do
			if i==5 or i==7 then
				--qArm[i] = sanitize(q, qOld[i])
			else
				qArm[i] = min(max(qMin[i], q), qMax[i])
			end
		end
		-- Add to the path
		table.insert(plan.qPath, qArm)
	until #plan.qPath > nStepsTimeout
	-- Finish
	local t1 = unix.time()
	-- Show the timing
	if Config.debug.armplan then
	  print(string.format(
			'%s%d steps (%d ms) {%.2fcm, %.2f°} [%s]',
			prefix, #plan.qPath, (t1-t0)*1e3,
			dTF[1]*1e2, dTF[2]*RAD_TO_DEG,
			#plan.qPath >= nStepsTimeout and 'Timeout' or 'Close'
		))
	end
	-- Update the goal (Filter the wrist position)
	plan.qGoal = self:find_shoulder(plan.tr, qArm, plan.weights)
	return libArmPlan.joint_preplan(self, plan)
end

-- Plan via Jacobian for waist and arm
function libArmPlan.jacobian_waist_preplan(self, plan)
	local prefix = string.format('jacobian_waist_preplan (%s) | ', self.id)
	assert(type(plan)=='table', prefix..'Bad plan')
	local qArm0 = assert(plan.qArm0, prefix..'Need initial arm')
	local qWaist0 = assert(plan.qWaist0, prefix..'Need initial waist')
	-- Find a guess of the final arm configuration
	local qWaistFGuess = plan.qWaistGuess or qWaist0
	local qArmFGuess, trGoal
	if type(plan.q)=='table' then
		trGoal = self.forward(plan.q, qWaistFGuess)
		qArmFGuess = plan.qArmGuess or plan.q
	elseif plan.tr then
		trGoal = plan.tr
		local weights = plan.weights
		qArmFGuess = plan.qArmGuess or self:find_shoulder(trGoal, qArm0, weights, qWaistFGuess)
	else
		error(prefix..'Need tr or q')
	end
	-- Use straight jacobian if no guess
	local qWaistArmFGuess
	if qArmFGuess then
		vector.new{qWaistFGuess[1], unpack(qArmFGuess)}
	else
		if Config.debug.armplan then
			print(prefix..'No guess found for the final!')
		end
	end
	-- Grab our limits
	local hz, dt = self.hz, self.dt
	local qMin = {-math.pi, unpack(self.qMin)}
	local qMax = {math.pi, unpack(self.qMax)}
	local dq_limit = {8*DEG_TO_RAD * dt, unpack(self.dq_limit)}
	-- Set the timing
	local timeout = assert(plan.timeout, prefix..'No timeout')
	local nStepsTimeout = math.ceil(timeout * hz)
	-- Initial position
	local qWaistArm = vector.new{qWaist0[1], unpack(qArm0)}
	-- Memory creation saving
	local dqdtNull = torch.Tensor(#qWaistArm)
	-- Begin
	local t0 = unix.time()
	local path = {}
	repeat
		-- Check if we are close enough
		local dp, drpy = get_distance(
			self, trGoal,
			{unpack(qWaistArm,2,#qWaistArm)}, {qWaistArm[1],0})
		local dTF = vector.new{vnorm(dp), vnorm(drpy)}
		-- Check if we are close enough
		if dTF[1] < 0.03 and dTF[2] < 5*DEG_TO_RAD then
			break
		end
		-- Form our desired velocity
		local vwTarget = {unpack(dp)}
		vwTarget[4], vwTarget[5], vwTarget[6] = unpack(drpy)
		-- Grab the joint velocities needed to accomplish the se(3) velocities
		local nullspace, J, Jinv = get_nullspace(self,
			{unpack(qWaistArm,2,#qWaistArm)},
			{qWaistArm[1], 0}
		)
		-- Find the motion and the null space
		local dqdtWaistArm = torch.mv(
			Jinv, torch.Tensor(vwTarget))

		-- Grab the velocities toward our guessed configuration, w/ or w/o null
		local dqdtCombo
		if qWaistArmFGuess then
			local dqNull = torch.Tensor(qWaistArm - qWaistArmFGuess)
			torch.mv(dqdtNull, nullspace, dqNull)
			dqdtCombo = dqdtWaistArm - dqdtNull:mul(nullFactor)
		else
			dqdtCombo = dqdtWaistArm
		end
		-- Respect the update rate, place as a lua table
		local dqCombo = vector.new(dqdtCombo:mul(dt))
		-- Check the speed limit usage
		local usage = {}
		for i, limit in ipairs(dq_limit) do
			table.insert(usage, fabs(dqCombo[i]) / limit)
		end
		local max_usage = max(unpack(usage))
		if max_usage > 1 then
			for i, dq in ipairs(dqCombo) do
				dqCombo[i] = dq / max_usage
			end
		end
		-- Apply the joint change (Creates a new table)
		qWaistArm = qWaistArm + dqCombo
		-- Check joint limit compliance
		for i, q in ipairs(qWaistArm) do
			if i==5 or i==7 then
				-- TODO: sanitize
			else
				qWaistArm[i] = min(max(qMin[i], q), qMax[i])
			end
		end
		-- Add to the path
		table.insert(path, qWaistArm)
	until #path > nStepsTimeout
	-- Show the timing
	local t1 = unix.time()
	if Config.debug.armplan then
	  print(string.format('%s: %d steps (%d ms)', prefix, #path, (t1-t0)*1e3))
	end


	local qPath = {}
	local wPath = {}
	for i, qWaistArmPlanned in ipairs(path) do
		qPath[i] = {unpack(qWaistArmPlanned, 2, #qWaistArmPlanned)}
		wPath[i] = {qWaistArmPlanned[1], 0}
	end
	plan.qPath = qPath
	plan.wPath = wPath

	-- Play the plan
	local qArmF, qWaistF = co_play_waist(plan)

	if #path==0 then return qArm0, qWaist0 end
	-- Hitting the timeout means we are done
	if #path >= nStepsTimeout then
		if Config.debug.armplan then print(prefix..'Timeout!', self.id, #path) end
		return qArmF, qWaistF
	end
	-- Goto the final
	local qArmF1 =
		self:find_shoulder(trGoal, qArmF, {0,1,0}, qWaistFGuess)
	if not qWaistArmF1 then
		if Config.debug.armplan then print(prefix..'No final solution found') end
		return qArmF, qWaistF
	end
	-- Use the pre-existing planner
	return libArmPlan.joint_waist_preplan(self, {
		q = qArmF1,
		qWaistGuess = qWaistFGuess,
		qArm0 = qArmF,
		qWaist0 = qWaistF,
		duration = 2
	})
end

-- Resume with an updated plan or empty table if no updates
function libArmPlan.jacobian_velocity(self, plan)
	local prefix = string.format('jacobian_preplan (%s) | ', self.id)
	-- Set the limits and check compliance
	local qMin, qMax = self.qMin, self.qMax
	local dq_limit = self.dq_limit
	-- While we have a plan, run the calculations
	local qArmSensed, qWaistSensed
	while type(plan)=='table' do
		-- Grab the joint velocities needed to accomplish the se(3) velocities
		local dqdtArm, nullspace = get_delta_qwaistarm(self, vwTarget, qArm)
		-- Grab the velocities toward our guessed configuration, w/ or w/o null
		local dqdtCombo
		if qArmFGuess then
			local dqdtNull = nullspace * torch.Tensor(qArm - qArmFGuess)
			dqdtCombo = dqdtArm - dqdtNull
		else
			dqdtCombo = dqdtArm
		end
		-- Respect the update rate, place as a lua table
		local dqCombo = vector.new(dqdtCombo:mul(dt))
		-- Check the speed limit usage
		local usage = {}
		for i, limit in ipairs(dq_limit) do
			table.insert(usage, fabs(dqCombo[i]) / limit)
		end
		local max_usage = max(unpack(usage))
		if max_usage > 1 then
			for i, dq in ipairs(dqCombo) do
				dqCombo[i] = dq / max_usage
			end
		end
		-- Apply the joint change (Creates a new table)
		qArm = qArm + dqCombo
		-- Check joint limit compliance
		for i, q in ipairs(qArm) do
			if i==5 or i==7 then
					-- TODO: Add sanitize
			else
				qArm[i] = min(max(qMin[i], q), qMax[i])
			end
		end
		-- Yield a command
		qArmSensed, qWaistSensed, plan = coroutine.yield()
	end
end

-- Resume with an updated plan or empty table if no updates
function libArmPlan.jacobian_wasit_velocity(self, plan)
	local prefix = string.format('jacobian_preplan (%s) | ', self.id)
	-- Set the limits
	local qMin = {-math.pi, unpack(self.qMin)}
	local qMax = {math.pi, unpack(self.qMax)}
	local dq_limit = {30*DEG_TO_RAD*self.dt, unpack(self.dq_limit)}
	-- While we have a plan, run the calculations
	local qArmSensed, qWaistSensed
	while type(plan)=='table' do
		-- Grab the joint velocities needed to accomplish the se(3) velocities
		local dqdtWaistArm, nullspace = get_delta_qwaistarm(
			self,
			vwTarget,
			{unpack(qWaistArm,2,#qWaistArm)},
			{qWaistArm[1], 0}
		)
		-- Grab the velocities toward our guessed configuration, w/ or w/o null
		local dqdtCombo
		if qWaistArmFGuess then
			local dqdtNull = nullspace * torch.Tensor(qWaistArm - qWaistArmFGuess)
			dqdtCombo = dqdtWaistArm - dqdtNull
		else
			dqdtCombo = dqdtWaistArm
		end
		-- Respect the update rate, place as a lua table
		local dqCombo = vector.new(dqdtCombo:mul(dt))
		-- Check the speed limit usage
		local usage = {}
		for i, limit in ipairs(dq_limit) do
			table.insert(usage, fabs(dqCombo[i]) / limit)
		end
		local max_usage = max(unpack(usage))
		if max_usage > 1 then
			for i, dq in ipairs(dqCombo) do
				dqCombo[i] = dq / max_usage
			end
		end
		-- Apply the joint change (Creates a new table)
		qWaistArm = qWaistArm + dqCombo
		-- Check joint limit compliance
		for i, q in ipairs(qWaistArm) do
			if i==5 or i==7 then
					-- TODO: Add sanitize
			else
				qWaistArm[i] = min(max(qMin[i], q), qMax[i])
			end
		end
		-- Yield a command
		qArmSensed, qWaistSensed, plan = coroutine.yield()
	end
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

	-- TODO: Check with SJ on the proper limits
	if self.id:lower():find'left' then
		print('left fix')
		qMin[2] = max(qMin[2], 0)
	end

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

local function pathJacobians(self, plan)
	-- Find the nullspace acting in
	local nulls = {}
	for i, q in ipairs(plan.qPath) do
		nulls[i] = get_nullspace(
			self, q, plan.wPath and plan.wPath[i])
	end
	plan.nulls = nulls
end

local function pathEigs(self, path)
	local eigs
	local eigVs = {}
	local eigVinvs = {}
	for i, nullspace in ipairs(path.nulls) do
		eigs, eigVs[i] = torch.eig(nullspace, 'V')
		eigVinvs[i] = torch.inverse(eigVs[i])
	end
	path.eigVs, path.eigVinvs = eigVs, eigVinvs
end

local opt_ch = require'simple_ipc'.new_requester('armopt')
local function optimize2(self, plan)
	local planName0 = os.tmpname()
	local planName = planName0..'.mat'
	assert(os.rename(planName0, planName), "Could not form tmp file")
	mattorch.saveTable(planName, plan)
	-- Send the tmpname of the mat file
	print('Sending plan:', planName)
	opt_ch:send(planName)
	local optResult = opt_ch:receive()
	print('Optimized!', planName)
	local optPath = mattorch.load(planName)
	-- Remove the file when done
	os.remove(planName)
	-- Place into a table
	-- TODO: Simpler way?
	local qOptimized0 = optPath.q:view(#plan.qPath, #plan.qGoal)
	local qOptimized = {}
	for i=1,#plan.qPath do
		qOptimized[i] = vector.new(qOptimized0[i])
	end
	return qOptimized
end

local function optimize(self, path)
	print('Optimization step', path.i_optimizations)
	--util.ptable(path)
	--print('===')
	local qPath = path.qPath
	local wPath = path.wPath
	local qGoal = path.qGoal
	local wGoal = path.wGoal
	local eigs = path.eigs
	local eigVs = path.eigVs
	local eigVinvs = path.eigVinvs
	local Js = path.Js
	local nulls = path.nulls
	local n = #qPath

	local qWaistArmGoal
	local nNull
	if type(wPath)=='table' and #wPath>0 then
		nNull = 2
		qWaistArmGoal = torch.Tensor{wGoal[1], unpack(qGoal)}
	else
		nNull = 1
		qWaistArmGoal = torch.Tensor(qGoal)
	end

	-- Distance to the goal all the time
	local dqGoal = {}
	local qWaistArm
	for i, q in ipairs(qPath) do
		if type(wPath)=='table' and #wPath>0 then
			qWaistArm = {wPath[i][1], unpack(q)}
		else
			qWaistArm = q
		end
		dqGoal[i] = torch.Tensor(qWaistArm):add(-1, qWaistArmGoal)
	end

	-- Find the coordinate in λ space
	--print('Finding the λ coordinates...')
	-- Setup the temporary variables
	local dqNull = torch.Tensor(#qWaistArmGoal)
	local dlambda = torch.Tensor(#qWaistArmGoal)
	local dλ = torch.Tensor(n, nNull)
	local λ2q = {}
	for i, q in ipairs(qPath) do
		local _λ2q = eigVs[i]:narrow(2, 1, nNull)
		λ2q[i] = _λ2q:clone()
		--print('λ2q', _λ2q)
		torch.mv(dqNull, nulls[i], dqGoal[i])
		--torch.mv(dqNull, nulls[i], dq[i])
		torch.mv(dlambda, eigVinvs[i], dqNull)
		local _dλ = dlambda:sub(1, nNull)
		dλ[i]:copy(_dλ)
		--print('dλ', _dλ)
	end

	-- Find the λ velocity gradient (accel)
	--[[
	local accelλ = {0}
	for i=2,#dλ-1 do
		accelλ[i] = (dλ[i+1] - dλ[i-1]) / 2
	end
	-- Not allowed to move the first coords
	table.insert(accelλ, 0)
	--]]

	--print('Running the kernel...')
	----[[
	local kernel = torch.Tensor(3, 1)
	kernel[1] = -1
	kernel[2] = 2
	kernel[3] = -1
	--]]

	--[[
	local kernel = torch.Tensor(5, 1)
	kernel[1] = -1
	kernel[2] = -3
	kernel[3] = 5
	kernel[4] = -3
	kernel[5] = -1
	--]]

	--[[
	local kernel = torch.Tensor(7, 1)
	kernel[1] = -1
	kernel[2] = -3
	kernel[3] = -5
	kernel[4] = 7
	kernel[5] = -5
	kernel[6] = -3
	kernel[7] = -1
	--]]

	----[[
	local jerkλ = torch.conv2(dλ, kernel, 'F')
		:narrow(1, 1+kernel:size(1)/2, dλ:size(1))
	--]]
	jerkλ[1]:add(dλ[1], -1, dλ[2])
	jerkλ[n]:add(dλ[n], -1, dλ[n-1])

	-- Find the λ acceleration gradient (jerk)
	--[[
	local _dλ = dλ:view(dλ:size(1))
	--print(dλ:size(), _dλ:size())
	-- NOTE: Could do the convolution only with the valid part
	local jerkλ = {
		_dλ[1] + -1*_dλ[2]
	}
	for i=2,n-1 do
		jerkλ[i] = 2*_dλ[i] + -1*_dλ[i-1] + -1*_dλ[i+1]
	end
	-- Not allowed to move the first coords
	table.insert(jerkλ, _dλ[n] + -1*_dλ[n-1])
	--]]

	-- Total gradient
	local wa = 1 /10
	local wj = 1 /100 * 0
	local gradλ = dλ:clone():mul(wa):add(wj, jerkλ)

	--[[
	local gradλ = {}
	for i=1, n do
		--gradλ[i] = wa * accelλ[i] + wj * jerkλ[i]
		--print(dλ[i], jerkλ[i])
		gradλ[i] = wa * _dλ[i] + wj * jerkλ[i]
	end
	--]]
	--accelλ = nil
	--jerkλ = nil

	-- Formulate the angular changes needed.
	-- Actually may need to change a bit?
	--[[
	local ddq = {}
	for i, g in ipairs(gradλ) do
		--ddq[i] = g * λ2q[i]
		ddq[i] = g * vector.new( λ2q[i]:view(λ2q[i]:size(1)) )
	end
	--]]

	----[[
	local a = 1
	local step = a * (1 - (path.i_optimizations-1)/path.n_optimizations)
	local ddq = {}
	for i, _λ2q in ipairs(λ2q) do
		local g = gradλ[i]
		ddq[i] = vector.new(
		torch.mv(_λ2q, g):mul(step)
			--:div(torch.norm(g)):mul(step)
			)
	end
	--]]

	--[[
	local ddq = {}
	local zeroQ = vector.zeros(nq)
	for i, dd in ipairs(ddq0) do
		nm = vector.norm(dd)
		ddq[i] = (nm<1e-9) and zeroQ or (step * dd / nm)
	end
	ddq0 = nil
	--]]

	-- Find the new path
	local qPathNew = {}
	local wPathNew = {}
	for i, d in ipairs(ddq) do
		if nNull==2 then
			--print('d[1]', d[1], vector.slice(d, 2))
			table.insert(qPathNew, qPath[i] - vector.slice(d, 2))
			table.insert(wPathNew, vector.new{wPath[i][1] - d[1], 0})
			--print('wPathNew', wPathNew[i])
		else
			print('d', d*RAD_TO_DEG)
			table.insert(qPathNew, qPath[i] - d)
		end
	end
	qPathNew[1] = qPath[1]
	if type(wPath)=='table' and #wPath>0 then
		wPathNew[1] = wPath[1]
	end
	--qPathNew[#qPath] = qPath[#qPath]
	return qPathNew, wPathNew, gradλ

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
		--
		optimize = optimize,
		optimize2 = optimize2,
		jacobians = pathJacobians,
		eigs = pathEigs,
	}
	return obj
end

return libArmPlan
