local state = {}
state._NAME = ...

local Body = require'Body'
local simple_ipc = require'simple_ipc'

--local timeout = 10.0
local t_entry, t_update, t_exit

-- Require all necessary fsm channels
local arm_ch    = simple_ipc.new_publisher('ArmFSM',true)
local motion_ch = simple_ipc.new_publisher('MotionFSM',true)
local lidar_ch  = simple_ipc.new_publisher('LidarFSM',true)

function state.entry()
  print(state._NAME..' Entry' )
  -- Update the time of entry
  local t_entry_prev = t_entry -- When entry was previously called
  t_entry = Body.get_time()
  t_update = t_entry

  -- TODO: Fix this
  --Body.set_lwheel_velocity(0);
  --Body.set_rwheel_velocity(0);
  ret=arm_ch:send'init'
  print(ret)
  ret=lidar_ch:send'start'
  print(ret)
  ret=motion_ch:send'stand'
  print(ret)
end

function state.update()
  --  print(state._NAME..' Update' ) 
  -- Get the time of update
  local t  = Body.get_time()
  local dt = t - t_update
  -- Save this at the last update time
  t_update = t
  --if t-t_entry > timeout then return'timeout' end

end

function state.exit()
  print(state._NAME..' Exit' )
  t_exit = Body.get_time()
end

return state