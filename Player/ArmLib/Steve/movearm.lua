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
		:set_chain(K.forward_larm, K.inverse_larm, K.jacobian)
		:set_limits(minLArm, maxLArm, radiansPerSecond)
		:set_update_rate(100)
		:set_shoulder_granularity(2*DEG_TO_RAD)
	rPlanner = P.new_planner('Right')
		:set_chain(K.forward_rarm, K.inverse_rarm, K.jacobian)
		:set_limits(minRArm, maxRArm, radiansPerSecond)
		:set_update_rate(100)
		:set_shoulder_granularity(2*DEG_TO_RAD)
end

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

local gen_via = {}
function gen_via.q(planner, goal, q0)
	if not goal then return end
	local co = coroutine.create(P.joint_preplan)
	if goal.tr then
		goal.q = planner:find_shoulder(goal.tr, q0, goal.weights)
		if not q then return 'via q | No target shoulder solution' end
	else
		goal.tr = planner.forward(goal.q)
	end
	local ok, msg = coroutine.resume(co, planner, goal.q, q0, goal.t)
	if not ok then co = msg end
	return co
end
function gen_via.jacobian(planner, goal, q0)
	if not goal then return end
	local co = coroutine.create(P.jacobian_preplan)
	if not goal.tr then goal.tr = planner.forward(goal.q) end
	local ok, msg = coroutine.resume(co, planner, goal.tr, q0, goal.weights, goal.t)
	if not ok then co = msg else goal.q = msg end
	return co
end
function gen_via.velocity(planner, goal, q0)
	if not goal then return end
	if not goal.vw then return end
	local co = coroutine.create(P.jacobian_velocity)
	local ok, msg = coroutine.resume(co, planner, goal.vw, q0, goal.t)
	if not ok then co = msg else goal.q = msg end
	return co
end

-- Take a desired joint configuration and move linearly in each joint towards it
function movearm.goto(l, r, add_compensation)
	local lco, rco
	local qLArm = Body.get_larm_command_position()
	local qRArm = Body.get_rarm_command_position()

	lco = l and type(gen_via[l.via])=='function' and gen_via[l.via](lPlanner, l, qLArm)
	rco = r and type(gen_via[r.via])=='function' and gen_via[r.via](rPlanner, r, qRArm)
	if type(lco)=='string' or type(rco)=='string' then
		print('goto | lco', lco)
		print('goto | rco', rco)
		return lco, rco
	end

	-- Add compensation
	local uTorsoComp
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

	return lco, rco, uTorsoComp
end

return movearm
