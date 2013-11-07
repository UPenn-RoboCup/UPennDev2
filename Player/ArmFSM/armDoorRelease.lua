local state = {}
state._NAME = ...
require'hcm'
local vector = require'vector'
local util   = require'util'
local movearm = require'movearm'
local stage = 1

function state.entry()
  print(state._NAME..' Entry' )
  -- Update the time of entry
  local t_entry_prev = t_entry
  t_entry = Body.get_time()
  t_update = t_entry 
  Body.set_lgrip_percent(0)
  Body.set_rgrip_percent(0)
  stage = 1
end

function state.update()
  local t  = Body.get_time()
  local dt = t - t_update
  -- Save this at the last update time
  t_update = t
  --if t-t_entry > timeout then return'timeout' end
  --  print(state._NAME..' Update' )
  -- Get the time of update

  local qLArm = Body.get_larm_command_position()
  local qRArm = Body.get_rarm_command_position()  

  return "done"; 
end

function state.exit()
  print(state._NAME..' Exit' )
end

return state