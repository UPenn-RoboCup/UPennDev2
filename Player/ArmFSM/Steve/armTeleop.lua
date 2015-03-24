local state = {}
state._NAME = ...
local Body = require'Body'
local movearm = require'movearm'

-- Compensation
-- 1: Use the compensation, but search for the shoulder
-- 2: Use the compenstation, and use the teleop shoulder options
local USE_COMPENSATION = 1

local t_entry, t_update, t_finish
local timeout = 10.0
local lPathIter, rPathIter
local qLGoal, qRGoal
local qLGoalFiltered, qRGoalFiltered
local qLD, qRD
local uTorso0, uTorsoComp
local loptions, roptions

function state.entry()
  print(state._NAME..' Entry' )
  -- Update the time of entry
  local t_entry_prev = t_entry
  t_entry = Body.get_time()
  t_update = t_entry
  -- Reset the human position
	local qcLArm = Body.get_larm_command_position()
	local qcRArm = Body.get_rarm_command_position()
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
	local qcLArm = Body.get_larm_command_position()
	local qcRArm = Body.get_rarm_command_position()
	--local qLArm = Body.get_larm_position()
	--local qRArm = Body.get_rarm_position()

	-- Check reset conditions
	if hcm.get_teleop_larm()~=qLGoal then lPathIter = nil end
	if hcm.get_teleop_rarm()~=qRGoal then rPathIter = nil end
	if hcm.get_teleop_compensation()~=USE_COMPENSATION then
		lPathIter = nil
		rPathIter = nil
		if USE_COMPENSATION==2 then
			-- Assume flip_roll is 0...
			hcm.set_teleop_loptions(loptions or {qcLArm[3], 0})
			hcm.set_teleop_roptions(roptions or {qcRArm[3], 0})
		end
	elseif USE_COMPENSATION==2 then
		if hcm.get_teleop_loptions()~=loptions then lPathIter = nil end
		if hcm.get_teleop_roptions()~=roptions then rPathIter = nil end
	end

  -- See if commanded a new position
  if not lPathIter or not rPathIter then
		-- Check if using the compensation
		USE_COMPENSATION = hcm.get_teleop_compensation()

    -- Get the goal from hcm
    qLGoal = hcm.get_teleop_larm()
    qRGoal = hcm.get_teleop_rarm()

		if USE_COMPENSATION > 0 then

			-- Grab the torso compensation
			local fkLComp, fkRComp
			fkLComp, fkRComp, uTorsoComp, uTorso0 =
				movearm.apply_q_compensation(qLGoal, qRGoal, movearm.get_compensation())

			-- Do we have desired null space options?
			if USE_COMPENSATION==2 then
				loptions = hcm.get_teleop_loptions()
				roptions = hcm.get_teleop_roptions()
				-- Form the iterator
				lPathIter, rPathIter, qLGoalFiltered, qRGoalFiltered, qLD, qRD =
					movearm.goto_tr_via_q(fkLComp, fkRComp, loptions, roptions)
			else
				-- Form the iterator
				lPathIter, rPathIter, qLGoalFiltered, qRGoalFiltered, qLD, qRD =
					movearm.goto_tr_via_q(fkLComp, fkRComp)
				loptions = qLGoalFiltered[3]
				roptions = qRGoalFiltered[3]
			end

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

	if USE_COMPENSATION > 0 then
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
	hcm.set_teleop_compensation(1)
end

return state
