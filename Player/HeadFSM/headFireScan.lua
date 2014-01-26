local Body = require'Body'
local t_entry, t_update
local state = {}
state._NAME = ...
require'hcm'

local util = require('util')

local scan_period = 3.0
local yawMag = 30*Body.DEG_TO_RAD

-- Neck limits
local dqNeckLimit = {45*Body.DEG_TO_RAD,45*Body.DEG_TO_RAD}

function state.entry()
  print(state._NAME..' Entry' ) 
  -- When entry was previously called
  local t_entry_prev = t_entry
  -- Update the time of entry
  t_entry = Body.get_time()

  hcm.set_fire_t(0)

  t_update = t_entry
  -- Reset the human position
  hcm.set_motion_headangle(Body.get_head_command_position())
end

function state.update()
  -- print(_NAME..' Update' )
  -- Get the time of update
  local t = Body.get_time()
  local dt = t - t_update

  local t_passed = t-t_entry
  local ph = (t_passed%scan_period)/scan_period


  -- Save this at the last update time
  t_update = t 

  neckAngleTarget = {-yawMag * 4*ph*yawMag,0}
  if ph>0.5 then neckAngleTarget[1] = yawMag - neckAngleTarget[1] end
  
  -- Grab where we are
  local qNeck = Body.get_head_command_position()
  -- Go!
  local qNeck_approach, doneNeck = 
    util.approachTol( qNeck, neckAngleTarget, dqNeckLimit, dt )
    
  -- Update the motors
  Body.set_head_command_position(qNeck_approach)

  local fire_t = hcm.get_fire_t()
  if fire_t - t < 0.5 then
    return "track"
  end
end

function state.exit()  
  print(state._NAME..' Exit' )
end

return state
