local state = {}
state._NAME = ...

local Body = require'Body'
local t_entry, t_update

function state.entry()
  print(state._NAME..' Entry' ) 
  -- When entry was previously called
  local t_entry_prev = t_entry
  -- Update the time of entry
  t_entry = Body.get_time()
  t_update = t_entry


  mag_sweep, t_sweep = unpack(vcm.get_mesh_sweep())
  min_pan = -mag_sweep/2
  max_pan = mag_sweep/2
end

function state.update()
--  print(_NAME..' Update' )
  -- Get the time of update
  local t = Body.get_time()
  local t_diff = t - t_update
  -- Save this at the last update time
  t_update = t

  local lidar_pos = Body.get_lidar_position()
  Body.set_lidar_torque_enable(1)
  if lidar_pos<0 then
   Body.set_lidar_command_position(min_pan) 
  else
   Body.set_lidar_command_position(max_pan) 
  end

end

function state.exit()
  print(state._NAME..' Exit' ) 
end

return state