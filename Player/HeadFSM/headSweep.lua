local state = {}
state._NAME = ...

local Body = require'Body'
local HT = require'libHeadTransform'
local util = require'util'

local t_entry, t_update
local dqNeckLimit = Config.fsm.dqNeckLimit
local tScan = Config.fsm.headSweep.tScan;
local yawMag = Config.head.yawMax;  --TODO: put in behavior
local dist = Config.fsm.headReady.dist;

-- min_eta_look = Config.min_eta_look or 2.0;

function state.entry()
  print(state._NAME..' entry');
  t_entry = Body.get_time();
  t_update = t_entry
  local headAngles = Body.get_head_position();
  if (headAngles[1] > 0) then
    direction = 1;
  else
    direction = -1;
  end
end

function state.update()
  local t = Body.get_time()
  local dt = t-t_update
  t_update = t

  local ph = (t-t_entry)/tScan;
  local yaw0 = direction * (ph - 0.5) * 2 * yawMag
  local yaw, pitch = HT.ikineCam(
  	dist * math.cos(yaw0), dist * math.sin(yaw0))

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
end

return state
