local movearm = {}
local Body = require'Body'
local util = require'util'
local vector = require'vector'
local T = require'Transform'
local P = require'libArmPlan'
-- Use Steve's kinematics for arm kinematics
local K = require'K_ffi'
-- Use SJ's kinematics for the mass properties
local K0 = Body.Kinematics

local dqLimit = DEG_TO_RAD / 3
local radiansPerSecond, torso0
do
	local degreesPerSecond = vector.new{15,15,15, 15, 25,25,25}
	--local degreesPerSecond = vector.new{15,10,20, 15, 20,20,20}
	--local degreesPerSecond = vector.ones(7) * 30
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
		:set_update_rate(100)
		:set_shoulder_granularity(2*DEG_TO_RAD)
	rPlanner = P.new_planner('Right')
		:set_chain(K.forward_rarm, K.inverse_rarm, K.jacobian_rarm)
		:set_limits(minRArm, maxRArm, radiansPerSecond)
		:set_update_rate(100)
		:set_shoulder_granularity(2*DEG_TO_RAD)
end
movearm.lPlanner = lPlanner
movearm.rPlanner = rPlanner

local function get_compensation(qcLArm, qcRArm, qcWaist)
	-- Legs are a bit different, since we are working in IK space
	local bH = mcm.get_stance_bodyHeight()
	local bT = mcm.get_stance_bodyTilt()
	local uTorso = mcm.get_status_uTorso()
	local aShiftX = mcm.get_walk_aShiftX()
  local aShiftY = mcm.get_walk_aShiftY()
	-- Current Foot Positions
	local uLeft = mcm.get_status_uLeft()
  local uRight = mcm.get_status_uRight()
	local zLeg = mcm.get_status_zLeg()
  local zSag = mcm.get_walk_zSag()
	local zLegComp = mcm.get_status_zLegComp()
	local zLeft, zRight = unpack(zLeg + zSag + zLegComp)
	local pLLeg = {uLeft[1],uLeft[2],zLeft,0,0,uLeft[3]}
  local pRLeg = {uRight[1],uRight[2],zRight,0,0,uRight[3]}
	qcWaist = qcWaist or {0, 0}

	-- Initial guess is torso0, our default position.
	-- See how far this guess is from our current torso position
	local uTorsoAdapt = util.pose_global(torso0, uTorso)
	local adapt_factor = 1.0
	for i=1,4 do
		-- Form the torso position now in the 6D space
		local pTorso = {uTorsoAdapt[1], uTorsoAdapt[2], bH, 0, bT, uTorsoAdapt[3]}
		local qLegs = K0.inverse_legs(pLLeg, pRLeg, pTorso, aShiftX, aShiftY)
		-- Grab the intended leg positions from this shift
		local qLLeg = {unpack(qLegs, 1, 6)}
		local qRLeg = {unpack(qLegs, 7, 12)}
		-- Calculate the COM position
		local com = K0.calculate_com_pos(qcWaist, qcLArm, qcRArm, qLLeg, qRLeg, 0, 0, 0)
		local uCOM = util.pose_global({com[1]/com[4], com[2]/com[4], 0}, uTorsoAdapt)
		uTorsoAdapt = uTorsoAdapt + adapt_factor * (uTorso - uCOM)
	end
	local uTorsoComp = util.pose_relative(uTorsoAdapt, uTorso)
	table.remove(uTorsoComp)
	return uTorsoComp
end

-- Take a desired joint configuration and move linearly in each joint towards it
function movearm.goto(l, r)
	-- Assume no motion
	local lco, rco = false, false

	local lplan = type(l)=='table' and P[l.via]
	if type(lplan)=='function' then
		lco = coroutine.create(lplan)
		local qLArm0 = l.qLArm0 or Body.get_larm_command_position()
		local qWaist0 = l.qWaist0 or Body.get_waist_command_position()
		local ok, msg = coroutine.resume(lco, lPlanner, l, qLArm0, qWaist0)
		if not ok then
			print('Error goto l |', msg)
		end
	end
	local rplan = type(r)=='table' and P[r.via]
	if type(rplan)=='function' then
		rco = coroutine.create(rplan)
		local qRArm0 = r.qRArm0 or Body.get_rarm_command_position()
		local qWaist0 = r.qWaist0 or Body.get_waist_command_position()
		local ok, msg = coroutine.resume(rco, rPlanner, r, qRArm0, qWaist0)
		if not ok then
			print('Error goto r |', msg)
		end
	end

	-- TODO: Add compensation again
	local uTorsoComp
	--[[
	if add_compensation then
		uTorsoComp = get_compensation(l.q, r.q, qcWaist)
		local trComp = T.trans(-uTorsoComp[1],-uTorsoComp[2], 0)
		-- Save the old transform, and update
		l.tr0 = l.tr
		r.tr0 = r.tr
		l.tr = trComp * l.tr0
		r.tr = trComp * r.tr0

		-- Re-run the planner
		lco = gen_via[l.via](lPlanner, l, qLArm)
		rco = gen_via[r.via](rPlanner, r, qRArm)
	end
			--]]

	return lco, rco
end

return movearm
