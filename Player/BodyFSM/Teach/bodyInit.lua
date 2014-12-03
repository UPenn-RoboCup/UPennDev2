local state = {}
state._NAME = ...

local Body = require'Body'

local timeout = 10.0
local t_entry, t_update, t_exit

local si = require'simple_ipc'
local motion_ch = si.new_publisher('MotionFSM!')

function state.entry()
  print(state._NAME..' Entry' )
  local t_entry_prev = t_entry
  t_entry = Body.get_time()
  t_update = t_entry
	--
  head_ch:send'init'
	arm_ch:send'init'
  motion_ch:send'stand'
  lidar_ch:send'pan'
end

function state.update()
  --  print(state._NAME..' Update' ) 
  -- Get the time of update
  local t  = Body.get_time()
  local dt = t - t_update
  -- Save this at the last update time
  t_update = t
  -- TODO: Investigate this...
  if t-t_entry > timeout then return'timeout' end
  --TODO: Check whether all FSMs have done initializing
  return 'done'
end

function state.exit()
  print(state._NAME..' Exit' )
  t_exit = Body.get_time()
end

return state
