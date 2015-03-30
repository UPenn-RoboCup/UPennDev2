local movearm = {}
local Body = require'Body'
local util = require'util'
local vector = require'vector'
local T = require'Transform'
local P = require'libPlan'
-- Use Steve's kinematics for arm kinematics
local K = require'K_ffi'
-- Use SJ's kinematics for the mass properties
local K0 = Body.Kinematics

local dqLimit = DEG_TO_RAD / 3
local radiansPerSecond, torso0
do
	local degreesPerSecond = vector.new{15,10,20, 15, 20,20,20}
	--local degreesPerSecond = vector.ones(7) * 30
	radiansPerSecond = degreesPerSecond * DEG_TO_RAD
	-- Compensation items
	local torsoX = Config.walk.torsoX
	torso0 = vector.new({-torsoX,0,0})
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
	--
	lPlanner = P.new_planner(minLArm, maxLArm, radiansPerSecond):set_chain(
		K.forward_larm, K.inverse_larm, 'larm')
	rPlanner = P.new_planner(minRArm, maxRArm, radiansPerSecond):set_chain(
		K.forward_rarm, K.inverse_rarm, 'rarm')
end

-- Take a desired joint configuration and move linearly in each joint towards it
function movearm.goto_q(qL, qR, safe)
	local lPathIter, rPathIter, iqLArm, iqRArm, qLDist, qRDist
	if qL then
		local qLArm = Body.get_larm_command_position()
		lPathIter, iqLArm, qLDist = lPlanner:joint_iter(qL, qLArm, safe)
	end
	if qR then
		local qRArm = Body.get_rarm_command_position()
		rPathIter, iqRArm, qRDist = rPlanner:joint_iter(qR, qRArm, safe)
	end
	return lPathIter, rPathIter, iqLArm, iqRArm, qLDist, qRDist
end

-- Take a desired Transformation matrix and move joint-wise to it
function movearm.goto_tr_via_q(trL, trR, loptions, roptions, lweights, rweights)
	local lPathIter, rPathIter, iqLArm, iqRArm, qLDist, qRDist
	local flipL, flipR
	if trL then
		local qcLArm = Body.get_larm_command_position()
		if loptions then
			iqLArm = lPlanner.inverse(trL, qcLArm, unpack(loptions))
		else
			iqLArm = lPlanner:find_shoulder(trL, qcLArm, lweights)
		end
		--print('L TR GOTO', vector.new(iqLArm))
		--print(trL)
		assert(iqLArm, 'L via q not found!')
		lPathIter, iqLArm, qLDist = lPlanner:joint_iter(iqLArm, qcLArm, true)
	end
	if trR then
		local qcRArm = Body.get_rarm_command_position()
		if roptions then
			iqRArm = rPlanner.inverse(trR, qcRArm, unpack(roptions))
		else
			iqRArm = rPlanner:find_shoulder(trR, qcRArm, rweights)
		end
		--print('R TR GOTO', vector.new(iqRArm))
		--print(trR)
		assert(iqLArm, 'R via q not found!')
		rPathIter, iqRArm, qRDist = rPlanner:joint_iter(iqRArm, qcRArm, true)
	end
	return lPathIter, rPathIter, iqLArm, iqRArm, qLDist, qRDist
end

-- Take a desired Transformation matrix and move in a line towards it
function movearm.goto_tr(trL, rwrist, loptions, roptions, lweights, rweights)
	local lPathIter, rPathIter
	if trL then
		local qcLArm = Body.get_larm_command_position()
		lPathIter, iqLArm, pLDist = lPlanner:line_iter(trL, qcLArm, loptions, lweights)
	end
	if rwrist then
		local qcRArm = Body.get_rarm_command_position()
		rPathIter, iqRArm, pRDist = rPlanner:line_iter(rwrist, qcRArm, roptions, rweights)
	end
	return lPathIter, rPathIter, iqLArm, iqRArm, pLDist, pRDist
end

-- Take a desired Transformation matrix and move in a line towards it
function movearm.goto_tr_stack(trL, rwrist, loptions, roptions, lweights, rweights)
	local lPathIter, rPathIter
	if trL then
		local qcLArm = Body.get_larm_command_position()
		lPathIter, iqLArm, pLDist = lPlanner:line_stack(trL, qcLArm, loptions, lweights)
	end
	if rwrist then
		local qcRArm = Body.get_rarm_command_position()
		rPathIter, iqRArm, pRDist = rPlanner:line_stack(rwrist, qcRArm, roptions, rweights)
	end
	return lPathIter, rPathIter, iqLArm, iqRArm, pLDist, pRDist
end

--[[
SJ's arm compensation:
calculate_com_pos -> get_torso_compensation -> get_next_movement -> plan_unified -> plan_arm_sequence -> armTeleop
--]]
function movearm.get_compensation()
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
	local zLeft,zRight = unpack(zLeg + zSag + zLegComp)
	local pLLeg = vector.new({uLeft[1],uLeft[2],zLeft,0,0,uLeft[3]})
  local pRLeg = vector.new({uRight[1],uRight[2],zRight,0,0,uRight[3]})
	--
	local qcLArm = Body.get_larm_command_position()
	local qcRArm = Body.get_rarm_command_position()
	local qcWaist = Body.get_waist_command_position()

	-- Initial guess is torso0, our default position.
	-- See how far this guess is from our current torso position
	local uTorsoAdapt = util.pose_global(torso0, uTorso)
	local adapt_factor = 1.0
	for i=1,4 do
		-- Form the torso position now in the 6D space
		local pTorso = vector.new{uTorsoAdapt[1], uTorsoAdapt[2], bH, 0, bT, uTorsoAdapt[3]}
		local qLegs = K0.inverse_legs(pLLeg, pRLeg, pTorso, aShiftX, aShiftY)
		-- Grab the intended leg positions from this shift
		local qLLeg = vector.slice(qLegs,1,6)
		local qRLeg = vector.slice(qLegs,7,12)
		-- Calculate the COM position
		local com = K0.calculate_com_pos(qcWaist, qcLArm, qcRArm, qLLeg, qRLeg, 0, 0, 0)
		local uCOM = util.pose_global(
			vector.new({com[1]/com[4], com[2]/com[4],0}),
			uTorsoAdapt
		)
		uTorsoAdapt = uTorsoAdapt + adapt_factor * (uTorso - uCOM)
	end
	return uTorsoAdapt, uTorso
end

function movearm.apply_q_compensation(qLGoal, qRGoal, uTorsoAdapt, uTorso0)
	local fkL = K.forward_larm(qLGoal)
	local fkR = K.forward_rarm(qRGoal)
	local uTorsoComp = util.pose_relative(uTorsoAdapt, uTorso0)
	local trComp = T.trans(-uTorsoComp[1],-uTorsoComp[2], 0)
	local fkLComp = trComp * fkL
	local fkRComp = trComp * fkR
	return fkLComp, fkRComp, uTorsoComp, uTorso0
end

-- uTorsoAdapt, uTorso0: global frame torso positions
function movearm.apply_tr_compensation(fkL, fkR, uTorsoAdapt, uTorso0)
	local uTorsoComp = util.pose_relative(uTorsoAdapt, uTorso0)
	local trComp = T.trans(-uTorsoComp[1],-uTorsoComp[2], 0)
	local fkLComp = trComp * fkL
	local fkRComp = trComp * fkR
	return fkLComp, fkRComp, uTorsoComp, uTorso0
end

-- list entry format: {tr, tr, }
-- Uses the quasi-static compensation
function movearm.path_iterators(list)
	-- return a coroutine
	return coroutine.wrap(function()
		for i, entry in ipairs(list) do
			local lGoal, rGoal, via, lw, rw = unpack(entry)
			local go = movearm[via]
			-- FK goals for use with copmensation
			local fkLComp, fkRComp
			if via:find'tr' then
				fkLComp, fkRComp, uTorsoComp, uTorso0 =
					movearm.apply_tr_compensation(lGoal, rGoal, movearm.get_compensation())
			else
				fkLComp, fkRComp, uTorsoComp, uTorso0 =
					movearm.apply_q_compensation(lGoal, rGoal, movearm.get_compensation())
			end
			coroutine.yield({go(fkLComp, fkRComp,nil,nil,lw,rw)}, uTorsoComp, uTorso0)
		end
	end)
end

return movearm
