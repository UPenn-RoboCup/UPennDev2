local Body = require'Body'
local util = require'util'
local vector = require'vector'
local t_entry, t_update
local state = {}
state._NAME = ...

-- TODO: need to compensate the torso pose
local headSpeed = {5 * DEG_TO_RAD, 6 * DEG_TO_RAD}
local lowAngle = {0, 45*DEG_TO_RAD}
local highAngle = {0, 0*DEG_TO_RAD}

local is_high
local desiredAngle
function state.entry()
  print(state._NAME..' Entry' )
  -- When entry was previously called
  local t_entry_prev = t_entry
  -- Update the time of entry
  t_entry = Body.get_time()
  t_update = t_entry
	desiredAngle = lowAngle
	is_high = true
end

function state.update()
--  print(_NAME..' Update' )
  -- Get the time of update
  local t = Body.get_time()
  local dt = t - t_update
  -- Save this at the last update time
  t_update = t

	local headNow = Body.get_head_command_position()
  local apprAng, doneHead = util.approachTol(headNow, desiredAngle, headSpeed, dt)
	Body.set_head_command_position(apprAng)

	if doneHead then
		desiredAngle = is_high and lowAngle or highAngle
		is_high = not is_high
	end

  return doneHead and 'done'
end

function state.exit()
  print(state._NAME..' Exit' )
end

return state
