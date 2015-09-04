local movearm = {}
local Body = require'Body'
local util = require'util'
local vector = require'vector'
local T = require'Transform'
local P = require'libArmPlan'
-- Use Steve's kinematics for arm kinematics
local K = require'K_ffi'
-- Use SJ's kinematics for the mass properties
--local K0 = Body.Kinematics

local dqLimit = DEG_TO_RAD / 3
local radiansPerSecond, torso0
do
	-- Good
	--local degreesPerSecond = vector.new{15,15,15, 15, 25,25,25}

	--local degreesPerSecond = vector.new{15,10,20, 15, 20,20,20}
	local degreesPerSecond = vector.ones(7) * 30
	--local degreesPerSecond = vector.ones(7) * 100
	radiansPerSecond = degreesPerSecond * DEG_TO_RAD
	-- Compensation items
	torso0 = {-Config.walk.torsoX, 0, 0}
end

local lPlanner, rPlanner
do
	local minLArm = vector.slice(
		Config.servo.min_rad, Config.parts.LArm[1], Config.parts.LArm[#Config.parts.LArm])
	local maxLArm = vector.slice(
		Config.servo.max_rad, Config.parts.LArm[1], Config.parts.LArm[#Config.parts.LArm])
	local minRArm = vector.slice(
		Config.servo.min_rad, Config.parts.RArm[1], Config.parts.RArm[#Config.parts.RArm])
	local maxRArm = vector.slice(
		Config.servo.max_rad, Config.parts.RArm[1], Config.parts.RArm[#Config.parts.RArm])
	-- Set up the planners for each arm
	print('Setting up planners')
	lPlanner = P.new_planner('Left')
		:set_chain(K.forward_larm, K.inverse_larm, K.jacobian_larm)
		:set_limits(minLArm, maxLArm, radiansPerSecond)
		:set_update_rate(10)
		:set_shoulder_granularity(2*DEG_TO_RAD)
	rPlanner = P.new_planner('Right')
		:set_chain(K.forward_rarm, K.inverse_rarm, K.jacobian_rarm)
		:set_limits(minRArm, maxRArm, radiansPerSecond)
		:set_update_rate(10)
		:set_shoulder_granularity(2*DEG_TO_RAD)
end
movearm.lPlanner = lPlanner
movearm.rPlanner = rPlanner

-- Play the plan as a coroutine
-- Array of joint angles
-- Callback on sensed data
local function co_play(self, plan, callback)

	-- Run the optimizer
	plan.n_optimizations = 4
	plan.update_jacobians = true
	--plan.n_optimizations = 0
	--plan.update_jacobians = false

	local t0 = unix.time()
	for i=1,plan.n_optimizations do
		plan.i_optimizations = i
		if not plan.nulls then
			self:jacobians(plan)
		end
		--if (not plan.eigVs) then self:eigs(plan) end
		--plan.qPath, plan.wPath = self:optimize(plan)
		-- Run another optimization...
		plan.qwPath = self:optimize2(plan)
		if plan.update_jacobians then
			self:jacobians(plan)
			--self:eigs(plan)
		end
	end
	local t1 = unix.time()
	print(string.format("%d iterations %.2f ms (%s)",
		plan.n_optimizations, (t1 - t0)*1e3, tostring(plan.via)))

	local qArmSensed, qWaistSensed = coroutine.yield()
	if type(callback)=='function' then
		callback(qArmSensed, qWaistSensed)
	end

	-- Check if we include the waist
	local hasWaist = plan.qWaistGuess~=nil
	local qArm, qWaist
	-- Try the optimized one
	for i, qWaistArm in ipairs(plan.qwPath) do
		if hasWaist then
			qArm = {unpack(qWaistArm, 2)}
			qWaist = {qWaistArm[1], 0}
		else
			qArm = qWaistArm
		end
		qArmSensed, qWaistSensed = coroutine.yield(qArm, qWaist)
		if type(callback)=='function' then
			callback(qArmSensed, qWaistSensed)
		end
	end
	-- TODO: This is a repeat...
	return qArm, qWaist
end

-- Take a desired joint configuration and move linearly in each joint towards it
function movearm.goto(l, r)
	-- Assume no motion
	local lco, rco = false, false

	local qcLArm = Body.get_larm_command_position()
	local qcRArm = Body.get_rarm_command_position()
	local qcWaist = Body.get_safe_waist_command_position()

	local lplan = type(l)=='table' and P[l.via]
	if type(lplan)=='function' then
		--lco = coroutine.create(lplan)

		-- must copy the plan to keep the wait ok
		l = util.shallow_copy(l)
		if l.q then vector.new(l.q) end
		if l.weights then vector.new(l.weights) end
		if l.qWaistGuess then vector.new(l.qWaistGuess) end
		if l.qArmGuess then vector.new(l.qArmGuess) end
		-- Check the various formats
		if l.tr then
			if #l.tr==6 then
				l.tr = T.transform6D(l.tr)
			elseif #l.tr==7 then
				l.tr = T.from_quatp(l.tr)
			end
		end
		l.qArm0 = vector.new(l.qLArm0 or qcLArm)
		l.qWaist0 = vector.new(l.qWaist0 or qcWaist)
		l.dt = lPlanner.dt
	end


	local rplan = type(r)=='table' and P[r.via]
	if type(rplan)=='function' then

		-- must copy the plan to keep the wait ok
		r = util.shallow_copy(r)

		-- Check the various formats
		if r.tr then
			if #r.tr==6 then
				r.tr = T.transform6D(r.tr)
			elseif #r.tr==7 then
				r.tr = T.from_quatp(r.tr)
			end
		end
		if r.weights then vector.new(r.weights) end
		if r.qWaistGuess then vector.new(r.qWaistGuess) end
		if r.qArmGuess then vector.new(r.qArmGuess) end
		r.qArm0 = vector.new(r.qRArm0 or qcRArm)
		r.qWaist0 = vector.new(r.qWaist0 or qcWaist)
		r.dt = rPlanner.dt
	end

	-- Add the compensation
	--[[
	local qLComp = qcLArm
	if l and l.tr and l.qArmFGuess then
		qLComp = l.qArmFGuess
	end
	local qRComp = qcRArm
	if r and r.tr and r.qArmFGuess then
		qRComp = r.qArmFGuess
	end
	local qWComp = qcWaist
	if l and l.tr and l.qWaistGuess then
		if r and r.tr and r.qWaistGuess then
			print('movearm | two waist comp')
		else
			qWComp = l.qWaistGuess
		end
	elseif r and r.tr and r.qWaistGuess then
		qWComp = r.qWaistGuess
	end

	if (l and l.tr) or (r and r.tr) then
		local uTorsoComp = Body.get_torso_compensation(qLComp, qRComp, qWComp)
		print('movearm | uTorsoComp', unpack(uTorsoComp))
		local trComp = T.trans(uTorsoComp[1], uTorsoComp[2], 0)
		if l and l.tr then
			print('movearm | Compensating L...')
			l.tr0 = l.tr
			l.tr = trComp * l.tr0
		end
		if r and r.tr then
			print('movearm | Compensating R...')
			r.tr0 = r.tr
			r.tr = trComp * r.tr0
		end
	end
	--]]


	if type(lplan)=='function' then
		--[[
		print('movearm | L Plan')
		util.ptable(l)
		print()
		--]]
		local ok, msg = pcall(lplan, lPlanner, l)
		if ok then
			lco = coroutine.create(co_play)
			ok, msg = coroutine.resume(lco, lPlanner, l)
		end
		if not ok then lco = msg end
	end
	if type(rplan)=='function' then
		--[[
		print('movearm | R Plan')
		util.ptable(r)
		print()
		--]]
		local ok, msg = pcall(rplan, rPlanner, r)
		if ok then
			rco = coroutine.create(co_play)
			ok, msg = coroutine.resume(lco, rPlanner, r)
		end
		if not ok then lco = msg end
	end

	return lco, rco
end

return movearm
