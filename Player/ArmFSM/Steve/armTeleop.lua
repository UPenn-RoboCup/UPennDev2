local state = {}
state._NAME = ...
local Body = require'Body'
local movearm = require'movearm'

local USE_COMPENSATION = true

local t_entry, t_update, t_finish
local timeout = 10.0
local lPathIter, rPathIter
local qLGoal, qRGoal
local qLGoalFiltered, qRGoalFiltered
local qLD, qRD
local uTorso0, uTorsoComp

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
	local qLArm = Body.get_larm_position()
	local qRArm = Body.get_rarm_position()

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
			local uTorsoAdapt, uTorso = movearm.get_compensation()
			uTorso0 = uTorso
			uTorsoComp = util.pose_relative(uTorsoAdapt, uTorso0)
			-- Apply the compensation
			local fkLComp, fkRComp = movearm.apply_compensation(qLGoal, qRGoal, uTorsoComp)
			-- Form the iterator
			lPathIter, rPathIter, qLGoalFiltered, qRGoalFiltered, qLD, qRD =
				movearm.goto_tr_via_q(fkLComp, fkRComp, {qLGoal[3]}, {qRGoal[3]})
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
