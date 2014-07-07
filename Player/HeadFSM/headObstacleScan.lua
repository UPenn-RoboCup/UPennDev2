local state = {}
state._NAME = ...

local Body = require'Body'
local util = require'util'
require'wcm'

local t_entry, t_update, stage
local dqNeckLimit = {25*DEG_TO_RAD,25*DEG_TO_RAD}
local tScan = Config.fsm.headObstacleScan.tScan
local yawMag = Config.fsm.headObstacleScan.yawMag

function state.entry()
	-- Enable obstacle detection
  wcm.set_obstacle_enable(1)
  print(state._NAME..' entry');
  t_entry = Body.get_time();
  t_update = t_entry
	stage = 1
--  local headAngles = Body.get_head_position();
--  if (headAngles[1] > 0) then
--    direction = 1;
--  else
--    direction = -1;
--  end
  print('Enable obstacle detection?', wcm.get_obstacle_enable())
end

function state.update()
  local t = Body.get_time()
  local dt = t-t_update
  t_update = t

  --local ph = (t-t_entry)/tScan;
  --local yaw = direction * (ph - 0.5) * 2 * yawMag

  local qNeck = Body.get_head_command_position()
  --25 deg can basically cover the whole field
  local pitch, yaw = 25*DEG_TO_RAD

	if stage == 1 then yaw = yawMag
	elseif stage == 2 then yaw = 0
	elseif stage == 3 then yaw = -yawMag
	elseif stage == 4 then yaw = 0
	else return 'done' end

  -- Grab where we are
  --local qNeck = Body.get_head_position()
  local qNeck_approach, doneNeck =
    util.approachTol(qNeck, {yaw, pitch}, dqNeckLimit, dt)

	if doneNeck then stage = stage+1 end

  -- Update the motors
  Body.set_head_command_position(qNeck_approach)

--  if t-t_entry > tScan then
--    return 'done'
--  end
end

function state.exit()
  wcm.set_obstacle_enable(0)
  print('Obstacle detection disabled?', wcm.get_obstacle_enable())
end

return state
