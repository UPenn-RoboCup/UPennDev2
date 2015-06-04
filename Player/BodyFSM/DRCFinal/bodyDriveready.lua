local state = {}
state._NAME = ...

local Body = require'Body'
local t_entry, t_update, t_exit

function state.entry()
  print(state._NAME..' Entry' )

  -- Update the time of entry
  local t_entry_prev = t_entry -- When entry was previously called
  t_entry = Body.get_time()
  t_update = t_entry

  --untorque arm and leg servos for car ingressing
  arm_ch:send'driveready'
  motion_ch:send'driveready'
--	gripper_ch:send'close' -- do we need anything for gripper?
  lidar_ch:send'drive' --center lidar
end

function state.update()
  --  print(state._NAME..' Update' ) 
  -- Get the time of update
  local t  = Body.get_time()
  local dt = t - t_update
  -- Save this at the last update time
  t_update = t
  --if t-t_entry > timeout then return'timeout' end

  --TODO: Check whether all FSMs have done initialzing 
  return 'done'

end

function state.exit()
  head_ch:send'drive'  --flip head
  print(state._NAME..' Exit' )
  t_exit = Body.get_time()
end

return state
