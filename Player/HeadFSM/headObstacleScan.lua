local state = {}
state._NAME = ...

local Body = require'Body'
local util = require'util'
require'wcm'

local t_entry, t_update
local dqNeckLimit = Config.fsm.dqNeckLimit
local tScan = Config.fsm.headObstacleScan.tScan
local yawMag = Config.fsm.headObstacleScan.yawMag

function state.entry()
  wcm.set_obstacle_enable(1)
  print(state._NAME..' entry');
  t_entry = Body.get_time();
  t_update = t_entry
  local headAngles = Body.get_head_position();
  if (headAngles[1] > 0) then
    direction = 1;
  else
    direction = -1;
  end
  print('Enable obstacle detection?', wcm.get_obstacle_enable())
end

function state.update()
  local t = Body.get_time()
  local dt = t-t_update
  t_update = t

  local ph = (t-t_entry)/tScan;
  local yaw = direction * (ph - 0.5) * 2 * yawMag
  --Based on webots, 25 deg can basically cover the whole field
  -- And according to rules, two obs will only be in the upper half
  local pitch = 25*DEG_TO_RAD

  -- Grab where we are
  local qNeck = Body.get_head_position()
  local qNeck_approach, doneNeck =
    util.approachTol(qNeck, {yaw, pitch}, dqNeckLimit, dt)
  -- Update the motors
  Body.set_head_command_position(qNeck_approach)

  if t-t_entry > tScan then
    return 'done'
  end
end

function state.exit()
  wcm.set_obstacle_enable(0)
  print('Obstacle detection disabled?', wcm.get_obstacle_enable())
end

return state
