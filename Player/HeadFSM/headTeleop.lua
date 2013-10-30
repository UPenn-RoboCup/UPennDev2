local Body = require'Body'
local t_entry, t_update
local state = {}
state._NAME = ...
require'hcm'

local util = require('util')

function state.entry()
  print(state._NAME..' Entry' ) 
  -- When entry was previously called
  local t_entry_prev = t_entry
  -- Update the time of entry
  t_entry = Body.get_time()
  t_update = t_entry

  -- TODO: What does this do?
  --lcm:set_head_lidar_panning(0)
end

function state.update()
  -- print(_NAME..' Update' )
  -- Get the time of update
  local t = Body.get_time()
  local dt = t - t_update
  -- Save this at the last update time
  t_update = t 

  local neckAngleTarget = hcm.get_motion_headangle()
--  print(unpack(neckAngleTarget))
  local dqNeckLimit = {45*Body.DEG_TO_RAD,45*Body.DEG_TO_RAD};

  local qNeck = Body.get_head_command_position()
  local qNeck_approach, doneNeck = 
    util.approachTol( qNeck, neckAngleTarget, dqNeckLimit, dt )
  Body.set_head_command_position(qNeck_approach)

  

end

function state.exit()  
  print(state._NAME..' Exit' )
end

return state