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

local degreesPerSecond = vector.new{15,10,20, 15, 20,20,20}
local radiansPerSecond = degreesPerSecond * DEG_TO_RAD
-- TODO: Add dt into the joint iterator
local dqLimit = DEG_TO_RAD / 3

-- Compensation items
local torsoX = Config.walk.torsoX
local torso0 = vector.new({-torsoX,0,0})

local minLArm = vector.slice(
	Config.servo.min_rad, Config.parts.LArm[1], Config.parts.LArm[#Config.parts.LArm]
)
local maxLArm = vector.slice(
	Config.servo.max_rad, Config.parts.LArm[1], Config.parts.LArm[#Config.parts.LArm]
)

local lPlanner = P.new_planner(minLArm, maxLArm, radiansPerSecond)
	:set_chain(K.forward_larm, K.inverse_larm)

local rPlanner = P.new_planner(
	vector.slice(Config.servo.min_rad, Config.parts.RArm[1], Config.parts.RArm[#Config.parts.RArm]),
	vector.slice(Config.servo.max_rad, Config.parts.RArm[1], Config.parts.RArm[#Config.parts.RArm]),
	radiansPerSecond
):set_chain(K.forward_rarm, K.inverse_rarm)

-- Take a desired joint configuration and move linearly in each joint towards it
function movearm.goto_q(lwrist, rwrist, safe)
	local lPathIter, rPathIter, iqLArm, iqRArm, qLDist, qRDist
	if lwrist then
		local qLArm = Body.get_larm_command_position()
		lPathIter, iqLArm, qLDist = lPlanner:joint_iter(lwrist, qLArm, dqLimit, safe)
	end
	if rwrist then
		local qRArm = Body.get_rarm_command_position()
		rPathIter, iqRArm, qRDist = rPlanner:joint_iter(rwrist, qRArm, dqLimit, safe)
	end
	return lPathIter, rPathIter, iqLArm, iqRArm, qLDist, qRDist
end

-- Take a desired Transformation matrix and move joint-wise to it
function movearm.goto_tr_via_q(trL, trR, loptions, roptions)
	local lPathIter, rPathIter, iqLArm, iqRArm, qLDist, qRDist
	if trL then
		local qcLArm = Body.get_larm_command_position()
		if loptions then
			iqLArm = K.inverse_larm(trL, qcLArm, unpack(loptions))
		else
			iqLArm = lPlanner:find_shoulder(trL, qcLArm)
		end
		lPathIter, iqLArm, qLDist = lPlanner:joint_iter(iqLArm, qcLArm, dqLimit, true)
	end
	if trR then
		local qcRArm = Body.get_rarm_command_position()
		iqRArm = K.inverse_rarm(trR, qcRArm, unpack(roptions or {}))
		rPathIter, iqRArm, qRDist = rPlanner:joint_iter(iqRArm, qcRArm, dqLimit, true)
	end
	return lPathIter, rPathIter, iqLArm, iqRArm, qLDist, qRDist
end

-- Take a desired Transformation matrix and move in a line towards it
function movearm.goto_tr(lwrist, rwrist, loptions, roptions)
	local lPathIter, rPathIter
	if lwrist then
		local qLArm = Body.get_larm_command_position()
		lPathIter = lPlanner:line_iter(lwrist, qLArm, 0.01, 3*DEG_TO_RAD, loptions)
	end
	if rwrist then
		local qRArm = Body.get_rarm_command_position()
		rPathIter = rPlanner:line_iter(rwrist, qRArm, 0.01, 3*DEG_TO_RAD, roptions)
	end
	return lPathIter, rPathIter
end

function movearm.goto_wrists(lwrist, rwrist)
	local lPathIter, rPathIter
	if lwrist then
	  local qLArm = Body.get_larm_position()
	  local qLWrist = Body.get_inverse_lwrist(qLArm, unpack(lwrist, 1, 2))
	  local qLGoal = Body.get_inverse_arm_given_wrist(qLWrist, lwrist[3])
		lPathIter = lPlanner:joint_iter(qLGoal, qLArm, dqLimit)
	end
	if rwrist then
		local qRArm = Body.get_rarm_position()
	  local qRWrist = Body.get_inverse_rwrist(qRArm, unpack(rwrist, 1, 2))
	  local qRGoal = Body.get_inverse_arm_given_wrist(qRWrist, rwrist[3])
		rPathIter = rPlanner:joint_iter(qRGoal, qRArm, dqLimit)
	end
	return lPathIter, rPathIter
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

function movearm.apply_compensation(qLGoal, qRGoal, uTorsoComp)
	local fkL = K.forward_larm(qLGoal)
	local fkR = K.forward_rarm(qRGoal)
	local trComp = T.trans(-uTorsoComp[1],-uTorsoComp[2], 0)
	local fkLComp = trComp * fkL
	local fkRComp = trComp * fkR
	return fkLComp, fkRComp
end

return movearm
