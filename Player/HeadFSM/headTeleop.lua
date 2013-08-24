local Body = require'Body'
local t_entry, t_update
local state = {}
state._NAME = ...
require('hcm')

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
  local t_diff = t - t_update
  -- Save this at the last update time
  t_update = t 

  local neck0 = hcm:get_control_head_movement()
  local neck = {neck0[1]*60*RAD, neck0[2]*60*RAD}
  Body.set_neck_target_position(neck)

end

function state.exit()
  -- TODO: What does this do?
  lcm:set_head_lidar_panning(0)
  print(state._NAME..' Exit' )
end

return state