local state = {}
state._NAME = ...
local Body = require'Body'
local movearm = require'movearm'

-- Compensation
-- 1: Use the compensation, but search for the shoulder
-- 2: Use the compenstation, and use the teleop shoulder options
local USE_COMPENSATION = 1

local t_entry, t_update, t_finish
local timeout = 30.0
local lPathIter, rPathIter
local qLGoal, qRGoal
local qLD, qRD
local uTorso0, uTorsoComp
local loptions, roptions


function state.entry()
  io.write(state._NAME, ' Entry' )
  -- Update the time of entry
  local t_entry_prev = t_entry
  t_entry = Body.get_time()
  t_update = t_entry
  -- Reset the human position
	local qcLArm0 = Body.get_larm_command_position()
	local qcRArm0 = Body.get_rarm_command_position()
	local teleopLArm = hcm.get_teleop_larm()
	local teleopRArm = hcm.get_teleop_rarm()

	lPathIter, rPathIter, qLGoal, qRGoal, qLD, qRD = movearm.goto_q(teleopLArm, teleopRArm, true)
	--print(state._NAME..' | qLGoal', qLGoal)
	--print(state._NAME..' | qRGoal', qRGoal)

end

function state.update()
	--io.write(state._NAME, ' Update' )
  -- Get the time of update
  local t  = Body.get_time()
  local dt = t - t_update
  -- Save this at the last update time
  t_update = t
  if t-t_entry > timeout then return'timeout' end

	-- Update our measurements availabl in the state
	local qcLArm = Body.get_larm_command_position()
	local qcRArm = Body.get_rarm_command_position()

	-- Timing necessary
	local moreL, q_lWaypoint = lPathIter(qcLArm, dt)
	local moreR, q_rWaypoint = rPathIter(qcRArm, dt)
	--io.write(moreL or -1,' ', moreR or -1)

	local qLNext = moreL and q_lWaypoint or qLGoal
	local qRNext = moreR and q_rWaypoint or qRGoal

	Body.set_larm_command_position(qLNext)
	Body.set_rarm_command_position(qRNext)

end

function state.exit()
  print(state._NAME..' Exit' )
end

return state
