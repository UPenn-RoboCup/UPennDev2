local state = {}
state._NAME = ...

local Body = require'Body'
local util = require'util'
require'wcm'

local t_entry, t_update, stage
local dqNeckLimit

function state.entry()
	-- Enable obstacle detection
  print(state._NAME..' entry');
  t_entry = Body.get_time();
  t_update = t_entry
	stage = 0
  dqNeckLimit = {50*DEG_TO_RAD, 50*DEG_TO_RAD}
end

function state.update()
  local t = Body.get_time()
  local dt = t-t_update
  t_update = t

  local yaw, pitch
	if stage == 0 then
    yaw = 0
    pitch = 80*DEG_TO_RAD
	elseif stage == 1 then
		yaw = 100*DEG_TO_RAD
    pitch = 30*DEG_TO_RAD
	elseif stage == 2 then
    yaw = -100*DEG_TO_RAD
    pitch = 30*DEG_TO_RAD
	elseif stage == 3 then
    yaw = 0
    pitch = 30*DEG_TO_RAD
	else
    return 'done'
	end

  -- Grab where we are
  local qNeck = Body.get_head_command_position()
  local qNeck_approach, doneNeck =
    util.approachTol(qNeck, {yaw, pitch}, dqNeckLimit, dt)
	if doneNeck then stage = stage+1 end

  -- Update the motors
  Body.set_head_command_position(qNeck_approach)
end

function state.exit()

end

return state
