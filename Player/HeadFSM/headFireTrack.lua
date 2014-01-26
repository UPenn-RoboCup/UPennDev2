local Body = require'Body'
local t_entry, t_update
local state = {}
state._NAME = ...
require'hcm'

local util = require('util')

-- Neck limits
local dqNeckLimit = {45*Body.DEG_TO_RAD,45*Body.DEG_TO_RAD}

function state.entry()
  print(state._NAME..' Entry' ) 
  -- When entry was previously called
  local t_entry_prev = t_entry
  -- Update the time of entry
  t_entry = Body.get_time()
  t_update = t_entry
  -- Reset the human position
  hcm.set_motion_headangle(Body.get_head_command_position())
end

function state.update()
  -- print(_NAME..' Update' )
  -- Get the time of update
  local t = Body.get_time()
  local dt = t - t_update
  -- Save this at the last update time
  t_update = t 

  -- Grab the target
  local neckAngleTarget = hcm.get_motion_headangle()
  --print('Neck angle',unpack(neckAngleTarget))
  -- Grab where we are
  local qNeck = Body.get_head_command_position()
  -- Go!
  local qNeck_approach, doneNeck = 
    util.approachTol( qNeck, neckAngleTarget, dqNeckLimit, dt )
    
  -- Update the motors
  Body.set_head_command_position(qNeck_approach)



  --TODO: restart scan afer some seconds

end

function state.exit()  
  print(state._NAME..' Exit' )
end

return state