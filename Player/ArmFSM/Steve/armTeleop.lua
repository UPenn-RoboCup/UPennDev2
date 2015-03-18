local state = {}
state._NAME = ...
local Body = require'Body'
local movearm = require'movearm'
-- Use SJ's kinematics for the mass properties
local K = Body.Kinematics
-- Use Steve's kinematics for arm kinematics
local K2 = require'K_ffi'
local T = require'Transform'

local USE_COMPENSATION = true

--[[
SJ's arm compensation:
calculate_com_pos -> get_torso_compensation -> get_next_movement -> plan_unified -> plan_arm_sequence -> armTeleop
--]]

local t_entry, t_update, t_finish
local timeout = 10.0
local lPathIter, rPathIter
local qLGoal, qRGoal
local qLGoalFiltered, qRGoalFiltered
local qcLArm, qcRArm, qcWaist, qLArm, qRArm, qWaist
local qLD, qRD
local uTorso0, uTorsoComp

-- Compensation items
local torsoX = Config.walk.torsoX
local torso0 = vector.new({-torsoX,0,0})

local function get_compensation()
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

	-- Initial guess is torso0, our default position.
	-- See how far this guess is from our current torso position
	local uTorsoAdapt = util.pose_global(torso0, uTorso)
	local adapt_factor = 1.0
	for i=1,4 do
		-- Form the torso position now in the 6D space
		local pTorso = vector.new{uTorsoAdapt[1], uTorsoAdapt[2], bH, 0, bT, uTorsoAdapt[3]}
		local qLegs = K.inverse_legs(pLLeg, pRLeg, pTorso, aShiftX, aShiftY)
		-- Grab the intended leg positions from this shift
		local qLLeg = vector.slice(qLegs,1,6)
		local qRLeg = vector.slice(qLegs,7,12)
		-- Calculate the COM position
		local com = K.calculate_com_pos(qcWaist, qcLArm, qcRArm, qLLeg, qRLeg, 0, 0, 0)
		local uCOM = util.pose_global(
			vector.new({com[1]/com[4], com[2]/com[4],0}),
			uTorsoAdapt
		)
		uTorsoAdapt = uTorsoAdapt + adapt_factor * (uTorso - uCOM)
	end
	return uTorsoAdapt, uTorso
end

local function compensate_arms(qLGoal, qRGoal, uTorsoComp)
	local fkL = K2.forward_larm(qLGoal)
	local fkR = K2.forward_rarm(qRGoal)
	local trComp = T.trans(-uTorsoComp[1],-uTorsoComp[2], 0)
	local fkLComp = trComp * fkL
	local fkRComp = trComp * fkR
	return movearm.goto_tr_via_q(fkLComp, fkRComp, {qLGoal[3]}, {qRGoal[3]})
end

function state.entry()
  print(state._NAME..' Entry' )
  -- Update the time of entry
  local t_entry_prev = t_entry
  t_entry = Body.get_time()
  t_update = t_entry
  -- Reset the human position
	qcLArm = Body.get_larm_command_position()
	qcRArm = Body.get_rarm_command_position()
  hcm.set_teleop_larm(qcLArm)
  hcm.set_teleop_rarm(qcRArm)
	-- Make sure to reset
	qLGoal, qRGoal = qcLArm, qcRArm
	lPathIter, rPathIter = nil, nil
end

function state.update()
	--  print(state._NAME..' Update' )
  -- Get the time of update
  local t  = Body.get_time()
  local dt = t - t_update
  -- Save this at the last update time
  t_update = t
  --if t-t_entry > timeout then return'timeout' end

	-- Update our measurements availabl in the state
	qcLArm = Body.get_larm_command_position()
	qcRArm = Body.get_rarm_command_position()
	qcWaist = Body.get_waist_command_position()
	qLArm = Body.get_larm_position()
	qRArm = Body.get_rarm_position()
	qWaist = Body.get_waist_position()

	-- Check reset conditions
	if hcm.get_teleop_larm()~=qLGoal then lPathIter = nil end
	if hcm.get_teleop_rarm()~=qRGoal then rPathIter = nil end

  -- See if commanded a new position
  if not lPathIter or not rPathIter then
    -- Get the goal from hcm
    qLGoal = hcm.get_teleop_larm()
    qRGoal = hcm.get_teleop_rarm()

		if USE_COMPENSATION then
			-- Grab the torso compensation
			local uTorsoAdapt, uTorso = get_compensation()
			uTorso0 = uTorso
			uTorsoComp = util.pose_relative(uTorsoAdapt, uTorso0)
			-- Form the iterator
			lPathIter, rPathIter, qLGoalFiltered, qRGoalFiltered, qLD, qRD =
				compensate_arms(qLGoal, qRGoal, uTorsoComp)
			--mcm.set_stance_uTorsoComp(uTorsoComp)
		else
			-- #nofilter
			qLGoalFiltered, qRGoalFiltered = qLGoal, qRGoal
			lPathIter, rPathIter, qLGoalFiltered, qRGoalFiltered, qLD, qRD =
				movearm.goto_q(qLGoal, qRGoal, true)
		end

  end

	-- Timing necessary
	local moreL, q_lWaypoint = lPathIter(qcLArm, dt)
	local moreR, q_rWaypoint = rPathIter(qcRArm, dt)
	-- No time needed
	--local moreL, q_lWaypoint = lPathIter(qLArm)
	--local moreR, q_rWaypoint = rPathIter(qRArm)

	local qLNext = moreL and q_lWaypoint or qLGoalFiltered
	local qRNext = moreR and q_rWaypoint or qRGoalFiltered

	local phaseL = moreL and moreL/qLD or 0
	local phaseR = moreR and moreR/qRD or 0

	if USE_COMPENSATION then
		local phase = math.max(phaseL, phaseR)
		local uTorsoNow = util.se2_interpolate(phase, uTorsoComp, uTorso0)
		mcm.set_stance_uTorsoComp(uTorsoNow)
	end

	-- Send to the joints
	Body.set_larm_command_position(qLNext)
	Body.set_rarm_command_position(qRNext)
  
end

function state.exit()  
  print(state._NAME..' Exit' )
end

return state
