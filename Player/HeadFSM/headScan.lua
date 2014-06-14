local state = {}
state._NAME = ...

local Body = require'Body'
require'wcm'
local t_entry, t_update

local dqNeckLimit = Config.fsm.dqNeckLimit
local pitch0 = Config.fsm.headScan.pitch0
local pitchMag = Config.fsm.headScan.pitchMag
local yawMag = Config.fsm.headScan.yawMag
local tScan = Config.fsm.headScan.tScan
local timeout = tScan * 2
local direction, pitchDir = 1, 1

function state.entry()
  print(state._NAME..' Entry' )
  -- When entry was previously called
  local t_entry_prev = t_entry
  -- Update the time of entry
  t_entry = Body.get_time()
  t_update = t_entry
end

function state.update()
  -- print(_NAME..' Update' )
  -- Get the time of update
  local t = Body.get_time()
  local dt = t - t_update
  -- Save this at the last update time
  t_update = t

  if t - t_entry > timeout then
    return 'timeout'
  end

	-- Check if we found the ball
  local ball_elapsed = t - wcm.get_ball_t()
  if ball_elapsed < 0.1 then
    return 'ballfound'
  end

  -- Find the phase
  local ph = (t - t_entry) / tScan
  ph = ph - math.floor(ph)

  local yaw, pitch
  if ph < 0.25 then --phase 0 to 0.25
    yaw = yawMag * (ph * 4) * direction
    pitch = pitch0 + pitchMag * pitchDir
  elseif ph < 0.75 then --phase 0.25 to 0.75
    yaw = yawMag * (1 - (ph - 0.25) * 4) * direction
    pitch = pitch0 - pitchMag * pitchDir
  else --phase 0.75 to 1
    yaw = yawMag * (-1 + (ph - 0.75) * 4) * direction
    pitch = pitch0 + pitchMag * pitchDir
  end
  Body.set_head_command_position({yaw, pitch})
end

function state.exit()
  print(state._NAME..' Exit')
end

return state

