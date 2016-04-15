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

local has_rrts, rrts = pcall(require, 'rrts')
print("Using rrts?", has_rrts, rrts)

local POS_THRESH = 0.01--0.025

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

local function get_pseudo_jacobian_dls(self, J, qWaistArm)
	-- Penalty for joint limits
	local qMin
	local qMax
	local qRange
	if #qWaistArm~=self.nq then
		qMin = self.qWMin
		qMax = self.qWMax
		qRange = self.qWRange
	else
		qMin = self.qMin
		qMax = self.qMax
		qRange = self.qRange
	end

	local l = {}
	for i, q in ipairs(qWaistArm) do
    l[i]= speed_eps + c * ((2*q - qMin[i] - qMax[i])/qRange[i]) ^ p
  end
	local JT = J:t()
  
  --print('get_pseudo_jacobian_dls', J:size(1), J:size(2), #l)
  
	local invInner = torch.inverse(
		torch.addmm(torch.diag(torch.Tensor(l)), JT, J)
	)
	local Jpseudoinv = torch.mm(invInner, JT)
	return Jpseudoinv
end

-- Use the Jacobian
local function get_nullspace(self, qWaistArm, qArm, qWaist)
	assert(type(qWaistArm)=='table',
		'get_delta_qwaistarm | Bad qWaistArm')
    -- TODO: This may be wrong, actually...? Unless relative to the waist... :/
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
	-- Initial points
	local qArm0
	local qWaist0
	if type(plan.qwPath)=='table' then
		-- Continue from a pre-existing path
		qArm0 = vector.new(plan.qwPath[#plan.qwPath])
		if #qArm0>self.nq then
			local diff = #qArm0 - self.nq
			qWaist0 = {unpack(qArm0, 1, diff + 1)}
			qArm0 = {unpack(qArm0, diff + 1)}
		end
	else
		qWaist0 = plan.qWaist0
		qArm0 = plan.qArm0
	end
	local qWaistArm0 = qArm0
	assert(type(qWaistArm0)=='table',
		prefix..'Bad initial configuration')
	vector.new(qWaistArm0)
	--  Set the Final Goal for the arm
	local qWaistGuess = plan.qWaistGuess
	local qWaistArmF
	if type(plan.q)=='table' then
		qWaistArmF = plan.q
	elseif plan.tr then
		qWaistArmF = self:find_shoulder(
			plan.tr, qArm0, plan.weights,
			qWaistGuess or qWaist0)
	end
	qWaistArmF = qWaistArmF or plan.qWaistArmGuess
	if type(qWaistArmF)~='table' then
		print(prefix..'No final goal')
		return plan
	end
	vector.new(qWaistArmF)
	-- Add waist motions
	if qWaistGuess and #qWaistArmF~=8 then
		assert(type(qWaist0)=='table')
		table.insert(qWaistArmF, 1, qWaistGuess[1])
		table.insert(qWaistArm0, 1, qWaist0[1])
	end
	-- Grab our limits
	local dq_limit
	local qMin
	local qMax
	if qWaistGuess then
		qMin = self.qWMin
		qMax = self.qWMax
		dq_limit = self.dqW_limit
	else
		qMin = self.qMin
		qMax = self.qMax
		dq_limit = self.dq_limit
	end
	-- Check compliance
	for i, q in ipairs(qWaistArmF) do
		qWaistArmF[i] = min(max(qMin[i], q), qMax[i])
	end
	-- Set the timeout
	local hz, dt = self.hz, self.dt
	-- Initial position
	local qArm = vector.copy(qArm0)
	local qWaist = vector.copy(qWaist0 or plan.qWaist0)
	local qWaistArm = vector.copy(qWaistArm0)
	-- Continue or start anew
	plan.qwPath = plan.qwPath or {}
	plan.nulls = plan.nulls or {}
	plan.Js = plan.Js or {}
	-- Task space path
	plan.vwPath = plan.vwPath or {}
	plan.vwEffective = plan.vwEffective or {}
	-- If given a duration, then check speed limit compliance
	if type(plan.duration)=='number' then
    print('qWaistArmF', qWaistArmF)
    print('qWaistArm0', qWaistArm0)
		local dqTotal = qWaistArmF - qWaistArm0
		local dqdtAverage = dqTotal / plan.duration
		local dqAverage = dqdtAverage * dt
		local usage = {}
		for i, limit in ipairs(dq_limit) do
			if fabs(dqAverage[i]) > limit then
				print(string.format(
					prefix.."dq[%d] |%g| > %g", i, dqAverage[i], limit))
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
		if Config.debug.armplan then
			print(prefix..'Using duration:', plan.duration)
			print(prefix..'Duration Steps:', nsteps)
		end
		for i=1,nsteps do
			qWaistArm = qWaistArm + dqAverage
			local nullspace, J, Jinv = get_nullspace(
				self, qWaistArm, qArm, qWaistGuess and qWaist)
			if plan.tr then
				local dp, drpy = get_distance(self, plan.tr, qArm, qWaist)
				local vwTarget0 = {
					dp[1], dp[2], dp[3],
					drpy[1], drpy[2], drpy[3],
				}
				table.insert(plan.vwPath, vwTarget0)
			end
			table.insert(plan.nulls, nullspace)
			table.insert(plan.Js, J)
			table.insert(plan.qwPath, qWaistArm)
			table.insert(plan.vwEffective,
				J * torch.Tensor(qWaistArm - (plan.qwPath[#plan.qwPath-1] or qWaistArm0)):div(dt)
			)
			-- Decompose for next step
			if qWaistGuess then
				qArm = {unpack(qWaistArm, 2)}
				qWaist = {qWaistArm[1], 0}
			else
				qArm = qWaistArm
				qWaist = qWaist0
			end
		end
		return plan
	end
	-- Timeout based
	local timeout = assert(plan.timeout, prefix..'No timeout')
	local nStepsTimeout = timeout * hz
	local n = 0
	repeat
		local dqF = qWaistArmF - qWaistArm
		local dist = vnorm(dqF)
		if dist < 0.5*DEG_TO_RAD then
			break
		end
		-- Check the speed limit usage
		local usage = {}
		for i, limit in ipairs(dq_limit) do
			table.insert(usage, fabs(dqF[i]) / limit)
		end
		local max_usage = max(unpack(usage))
		if max_usage>1 then
			for i, qF in ipairs(dqF) do
				dqF[i] = qF / max_usage
			end
		end
		-- Apply the joint change
		qWaistArm = qWaistArm + dqF
		local nullspace, J, Jinv = get_nullspace(
			self, qWaistArm, qArm, qWaistGuess and qWaist)
		if plan.tr then
			local dp, drpy = get_distance(self, plan.tr, qArm, qWaist)
			local vwTarget0 = {
				dp[1], dp[2], dp[3],
				drpy[1], drpy[2], drpy[3],
			}
			table.insert(plan.vwPath, vwTarget0)
		end
		table.insert(plan.nulls, nullspace)
		table.insert(plan.Js, J)
		table.insert(plan.qwPath, qWaistArm)
		table.insert(plan.vwEffective,
			J * torch.Tensor(qWaistArm - (plan.qwPath[#plan.qwPath-1] or qWaistArm0)):div(dt)
		)
		-- Decompose for next step
		if qWaistGuess then
			qArm = {unpack(qWaistArm, 2)}
			qWaist = {qWaistArm[1], 0}
		else
			qArm = qWaistArm
			qWaist = qWaist0
		end
		n = n + 1
	until n > nStepsTimeout

	if Config.debug.armplan then
		if n > nStepsTimeout then
			print(prefix..'Timeout: ', n)
		else
			print(prefix, n..' steps')
		end
	end
	return plan
end

-- Plan via Jacobian for just the arm
function libArmPlan.jacobian_preplan(self, plan)
	local prefix = string.format('jacobian_preplan (%s) | ', self.id)
	assert(type(plan)=='table', prefix..'Bad plan')
	-- Initial points
	local qArm0
	local qWaist0
	if type(plan.qwPath)=='table' then
		-- Continue from a pre-existing path
		qArm0 = plan.qwPath[#plan.qwPath]
		if #qArm0>self.nq then
			local diff = #qArm0 - self.nq
			qWaist0 = vector.new{unpack(qArm0, 1, diff + 1)}
			qWaist0 = vector.new{unpack(qArm0, diff + 1)}
		end
	else
		qArm0 = plan.qArm0
		qWaist0 = plan.qWaist0
	end
	assert(qArm0, prefix..'Need initial arm')
	assert(qWaist0, prefix..'Need initial waist')
	-- Update a guess for the final configuration
	local qWaistGuess = plan.qWaistGuess
	local qArmGuess = plan.qArmGuess
	local qWaistArmGuess
	if type(plan.q)=='table' then
		plan.tr = self.forward(plan.q, qWaistGuess or qWaist0)
		qWaistArmGuess = plan.q
	elseif plan.tr then
		qWaistArmGuess = qArmGuess or self:find_shoulder(
			plan.tr, qArm0, plan.weights, qWaistGuess or qWaist0)
	end
	assert(type(plan.tr)=='table',
		prefix..'No goal specified')
	-- Use straight jacobian if no guess
	if qWaistArmGuess then
		plan.qWaistArmGuess = vector.new(qWaistArmGuess)
		if qWaistGuess then
			table.insert(qWaistArmGuess, 1, qWaistGuess[1])
		end
	else
		if Config.debug.armplan then
			print(prefix..'No guess found for the final!')
		end
	end
	-- Set the timing
	local timeout = plan.timeout or 10
	local hz, dt = self.hz, self.dt
	local nStepsTimeout = math.ceil(timeout * hz)
	-- Grab our limits
	local dq_limit
	local qMin
	local qMax
	if qWaistGuess then
		qMin = self.qWMin
		qMax = self.qWMax
		dq_limit = self.dqW_limit
	else
		qMin = self.qMin
		qMax = self.qMax
		dq_limit = self.dq_limit
	end
	-- Initial position
	local qArm = vector.copy(qArm0)
	local qWaist = vector.copy(qWaist0)
	local qWaistArm
	if qWaistGuess then
		qWaistArm = vector.new{qWaist[1], unpack(qArm)}
	else
		qWaistArm = vector.copy(qArm)
	end
	local qWaistArm0 = qWaistArm
	-- Null space variables
	local nullFactor = 0.3 --0.2
	local dqdtNull = torch.Tensor(#qWaistArm)
	-- Continue the path
	plan.nulls = plan.nulls or {}
	local anull = 0.5
	plan.Js = plan.Js or {}
	plan.qwPath = plan.qwPath or {}
	-- Task space path
	plan.vwPath = plan.vwPath or {}
	plan.vwEffective = plan.vwEffective or {}
	-- Begin
	print(prefix..'Beginning')
	local n = 0
	local dTF
  
  if plan.interactions then print('Have interactions!') end
  
	local t0 = unix.time()
	repeat
		local dp, drpy =
			get_distance(self, plan.tr, qArm, qWaist)
		if n==0 then print('Dist', vector.new(dp), vector.new(drpy)) end
		-- Check if we are within threshold
		dTF = {vnorm(dp), vnorm(drpy)}
		if dTF[1] < POS_THRESH and dTF[2] < 3*DEG_TO_RAD then
			break
		end
		-- Form our desired velocity
		local vwTarget0 = {
			dp[1], dp[2], dp[3],
			drpy[1], drpy[2], drpy[3],
		}
		local vwTarget = torch.Tensor(vwTarget0)
		-- Find the nullspace and Jacobian
		local nullspace, J, Jinv = get_nullspace(
			self, qWaistArm, qArm, qWaistGuess and qWaist)
			--[[
		nullspace = torch.mul(nullspace, anull):add(
			torch.mul(plan.nulls[#plan.nulls] or nullspace, 1-anull)
		)
		--]]
		-- Joint velocities to accomplish the se(3) velocities
		local dqdtArm = torch.mv(Jinv, vwTarget)
		local dqdtCombo
		--print('here...')
		--print('qWaistArm', qWaistArm)
		--print('qWaistArmGuess', qWaistArmGuess)
    if plan.gamma then
      local qTarget = plan.gamma<0 and self.qMin or self.qMax
      --print('qTarget', qTarget)
			local dqNull = torch.Tensor( qWaistArm - qWaistArmGuess )
			torch.mv(dqdtNull, nullspace, dqNull)
			dqdtCombo = dqdtArm - dqdtNull:mul(nullFactor)
    elseif plan.interactions and #plan.interactions>0 then
      local dqdtNullSum = torch.Tensor(qWaistArm):zero()
			for _, qInteraction in ipairs(plan.interactions) do
        local dqNull = torch.Tensor(qWaistArm - qInteraction)
  			torch.mv(dqdtNull, nullspace, dqNull)
        dqdtNullSum = dqdtNullSum + dqdtNull
      end
      dqdtNullSum = dqdtNullSum:mul(1/#plan.interactions)
			dqdtCombo = dqdtArm - dqdtNullSum:mul(nullFactor)
		elseif qWaistArmGuess then
			local dqNull = torch.Tensor(
				qWaistArm - qWaistArmGuess)
			torch.mv(dqdtNull, nullspace, dqNull)
			dqdtCombo = dqdtArm - dqdtNull:mul(nullFactor)
		else
			dqdtCombo = dqdtArm
		end
		-- Respect the update rate, place as a lua table
		local dqCombo = vector.new(dqdtCombo) * dt
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
		local qOld = qWaistArm
		qWaistArm = qWaistArm + dqCombo
		-- Check joint limit compliance
		for i, q in ipairs(qWaistArm) do
			qWaistArm[i] = min(max(qMin[i], q), qMax[i])
		end
		-- Add to the path
		table.insert(plan.qwPath, qWaistArm)
		table.insert(plan.nulls, nullspace)
		table.insert(plan.Js, J)
		table.insert(plan.vwPath, vwTarget0)
		table.insert(plan.vwEffective,
			J * torch.Tensor(qWaistArm - (plan.qwPath[#plan.qwPath-1] or qWaistArm0)):div(dt)
		)
		-- Decompose for next iteration
		if qWaistGuess then
			qArm = {unpack(qWaistArm, 2)}
			qWaist = {qWaistArm[1], 0}
		else
			qArm = qWaistArm
			qWaist = qWaist0
		end
		n = n + 1
	until n > nStepsTimeout
	-- Finish
	local t1 = unix.time()
	-- Show the timing
	if Config.debug.armplan then
	  print(string.format(
			'%s%d steps (%d ms) {%.2fcm, %.2f°} [%s]',
			prefix, n, (t1-t0)*1e3,
			dTF[1]*1e2, dTF[2]*RAD_TO_DEG,
			n >= nStepsTimeout and 'Timeout' or 'Close'
		))
	end
	if n >= nStepsTimeout then
		return libArmPlan.joint_preplan(self, plan)
	else
		return plan
	end
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
		if qWaistArmGuess then
			local dqdtNull = nullspace * torch.Tensor(qWaistArm - qWaistArmGuess)
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

function libArmPlan.rrt_star(self, plan)
  
	local prefix = string.format('rrt_star (%s) | ', self.id)
	assert(type(plan)=='table', prefix..'Bad plan')
  
	-- Update a guess for the final configuration
	if type(plan.q)=='table' then
		plan.tr = self.forward(plan.q, plan.qWaistGuess or plan.qWaist0)
		plan.qWaistArmGuess = vector.copy(plan.q)
	elseif plan.tr then
		plan.qWaistArmGuess = plan.qArmGuess or self:find_shoulder(
			plan.tr, plan.qArm0, plan.weights, plan.qWaistGuess or plan.qWaist0)
    vector.new(plan.qWaistArmGuess)
	end
	assert(type(plan.tr)=='table', prefix..'No goal specified')

	-- Append the waist to the guess if need be
  if plan.qWaistArmGuess and plan.qWaistGuess then
		table.insert(plan.qWaistArmGuess, 1, plan.qWaistGuess[1])
	end
  
  if not plan.qWaistArmGuess then
    print('No guess!')
  else
    print('Guess:', plan.qWaistArmGuess)
  end
  
  plan.qwPath = rrts.plan(plan, self, 1.5, 1e4)
  
  if type(plan.qwPath)=='table' then
    print('Found:', #plan.qwPath)
    
    local p = {}
    for i=2,#plan.qwPath do
      print('q',i, vector.new(plan.qwPath[i]))
      p = libArmPlan.joint_preplan(self, {
        qArm0 = plan.qwPath[i-1],
        qWaist0 = {0,0},
        q = plan.qwPath[i],
        duration = 6,
        timeout = 7,
        qwPath = p.qwPath
        })
    end
    -- Finish the path...
    if plan.qWaistArmGuess then
      p = libArmPlan.joint_preplan(self, {
        qArm0 = plan.qwPath[#plan.qwPath],
        qWaist0 = {0,0},
        q = plan.qWaistArmGuess,
        duration = 6,
        timeout = 7,
        qwPath = p.qwPath
        })
    end
    
    print('New qwPath', p.qwPath)
    print('#', #p.qwPath)
    plan.qwPath = p.qwPath
    
  end
  
  return plan
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
	self.qMin = vector.new(assert(qMin))
	self.qMax = vector.new(assert(qMax))

	-- TODO: Check with SJ on the proper limits
	if self.id:lower():find'left' then
		print('left fix')
		qMin[2] = max(qMin[2], 0)
	end

	self.dqdt_limit = assert(dqdt_limit)
	self.qRange = qMax - qMin
  self.qMid = self.qMin /2 + self.qMax /2

	print(self.id, 'mid', self.qMid)

	-- Add the waist
	-- TODO: Make better API :P
	self.qWMin = vector.new{-math.pi/2, unpack(self.qMin)}
	self.qWMax = vector.new{math.pi/2, unpack(self.qMax)}
	self.qWRange = self.qWMax - self.qWMin
  self.qWMid = self.qWMin /2 + self.qWMax /2
	self.dqWdt_limit = vector.new{
		10*DEG_TO_RAD, unpack(self.dqdt_limit)}

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
	self.dqW_limit = self.dqWdt_limit / hz
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

-- TODO: Also add the vwTarget
local function pathJacobians(self, plan)
	-- Find the nullspace acting in
	local nulls = {}
	local Js = {}
	local vwPath = {}
	local vwEffective = {}
	local qArm, qWaist
	for i, qw in ipairs(plan.qwPath) do
    vector.new(qw)
		-- Decompose
		if #qw>self.nq then
			qArm = {unpack(qw, 2)}
			qWaist = {qw[1], 0}
		else
			qArm = qw
			qWaist = nil
		end
		nulls[i], Js[i] =
			get_nullspace(self, qw, qArm, plan.qWaistGuess and qWaist)
		table.insert(vwEffective,
			Js[i] * torch.Tensor(qw - (plan.qwPath[i-1] or plan.qwPath[1])):div(self.dt)
		)
		if plan.tr then
			local dp, drpy = get_distance(self, plan.tr, qArm, qWaist)
			local vwTarget0 = {
				dp[1], dp[2], dp[3],
				drpy[1], drpy[2], drpy[3],
			}
			table.insert(vwPath, vwTarget0)
		end
	end
	plan.nulls = nulls
	plan.Js = Js
	plan.vwPath = vwPath
	plan.vwEffective = vwEffective
end

local opt_ch = require'simple_ipc'.new_requester('armopt')
local ok, mattorch = pcall(require, 'mattorch')
local function optimize(self, plan, stop)
	local planName0 = os.tmpname()
	local planName = planName0..'.mat'
	assert(os.rename(planName0, planName), "Could not form tmp file")
	-- 0 is the q version, 1 is the lambda version
	plan.stop = stop and 1 or 0
	--plan.kind = 1
	plan.kind = 0
	-- For debugging
	--[[
	local np = #plan.qwPath
	local nq = #plan.qwPath[1]
	local nNull = nq - 6
	local qGoal = plan.qwPath[np]
	local dλ = torch.Tensor(np, nNull)
	local Us = {}
	for i, q in ipairs(plan.qwPath) do
		local dqGoal = torch.Tensor(q - qGoal)
		local U, S, V = torch.svd(plan.nulls[i])
		local subV = V:sub(1, nq, 1, nNull):t()
		dλ[i] = torch.diag(S):sub(1,nNull,1,nNull) * subV * dqGoal
		Us[i] = U
	end
	plan.dlambda0 = dλ
	--]]
	mattorch.saveTable(planName, plan)
	-- Send the tmpname of the mat file
	print('Sending plan:', planName)
	opt_ch:send(planName)
	local optResult = opt_ch:receive()
	print('Optimized!', planName)
	local optPath = mattorch.load(planName)
	--os.exit()
	-- Remove the file when done
	os.remove(planName)
	--if stop then return end
	--util.ptable(optPath)
	-- Place into a table
	local optPath = optPath.qw or optPath.qLambda
	assert(optPath, 'No optimized path!')
	--print('optPath', optPath:size())
	-- MATLAB and torch are transposed...
	local qOptimized0 = optPath:t()
	local qOptimized = {}
	for i=1,#plan.qwPath do
		qOptimized[i] = vector.new(qOptimized0[i])
	end
	return qOptimized
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
    --
    get_nullspace = get_nullspace,
    get_distance = get_distance
	}
	return obj
end

return libArmPlan
