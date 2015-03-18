local state = {}
state._NAME = ...
local Body = require'Body'
local movearm = require'movearm'
-- Use SJ's kinematics for the mass properties
local K = Body.Kinematics

--[[
SJ's arm compensation:
calculate_com_pos -> get_torso_compensation -> get_next_movement -> plan_unified -> plan_arm_sequence -> armTeleop
--]]

local t_entry, t_update, t_finish
local timeout = 10.0
local lPathIter, rPathIter
local qLGoal, qRGoal
local qcLArm, qcRArm, qcWaist, qLArm, qRArm, qWaist

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
	return util.pose_relative(uTorsoAdapt, uTorso)
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
  qLGoal = hcm.get_teleop_larm()
  qRGoal = hcm.get_teleop_rarm()
  lPathIter, rPathIter = movearm.goto_q(qLGoal, qRGoal)
end

function state.update()
	--  print(state._NAME..' Update' )
  -- Get the time of update
  local t  = Body.get_time()
  local dt = t - t_update
  -- Save this at the last update time
  t_update = t
  --if t-t_entry > timeout then return'timeout' end

  -- See if commanded a new position
  if qLGoal~=hcm.get_teleop_larm() or qRGoal~=hcm.get_teleop_rarm() then
    -- Get the goal from hcm
    qLGoal = hcm.get_teleop_larm()
    qRGoal = hcm.get_teleop_rarm()
    -- Make the iterators, sanitize the output
    lPathIter, rPathIter = movearm.goto_q(qLGoal, qRGoal, true)
  end
  
	-- Update our measurements availabl in the state
	qcLArm = Body.get_larm_command_position()
	qcRArm = Body.get_rarm_command_position()
	qcWaist = Body.get_waist_command_position()

	qLArm = Body.get_larm_position()
	qRArm = Body.get_rarm_position()
	qWaist = Body.get_waist_position()

	-- Find the compenstation needed
	local comp = get_compensation()
	print('Compensation', comp)

	-- Timing necessary
	local moreL, q_lWaypoint = lPathIter(qcLArm, dt)
	-- No time needed
	--local moreL, q_lWaypoint = lPathIter(qLArm)
	Body.set_larm_command_position(moreL and q_lWaypoint or qLGoal)
	
	-- Timing
	local moreR, q_rWaypoint = rPathIter(qRArm, dt)
	-- No time
	--local moreR, q_rWaypoint = rPathIter(qRArm)
	--if moreR then print(moreR, qRArm, q_rWaypoint) end
	Body.set_rarm_command_position(moreR and q_rWaypoint or qRGoal)
  
end

function state.exit()  
  print(state._NAME..' Exit' )
end

return state
