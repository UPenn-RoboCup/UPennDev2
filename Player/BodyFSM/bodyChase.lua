local Config = require'Config'
local Body   = require'Body'
require'hcm'

local state = {}
state._NAME = 'bodyTeleop'

local t_entry, t_update, t_exit
local timeout = 10.0

-- Talk to the walk engine
local simple_ipc = require'simple_ipc'
local walk_ch = simple_ipc.new_publisher('Walk',true)

function state.entry()
  print(state._NAME..' Entry' )
  -- Update the time of entry
  local t_entry_prev = t_entry -- When entry was previously called
  t_entry = Body.get_time()
  t_update = t_entry
end

function state.update()
  -- print(state._NAME..' Update' )
  -- Get the time of update
  local t  = Body.get_time()
  local dt = t - t_update
  -- Save this at the last update time
  t_update = t
  -- Default timeout exit strategy
  if t-t_entry > timeout then return'timeout' end

  -- Get our pose
  local pose = wcm.get_slam_pose()

  -- Get the human set (global/relative?) destination
  local dest  = hcm.get_navigation_dest()
  local x,y,a = unpack(dest)
  local destR = math.sqrt(x^2 + y^2)

  -- Propagate to shared memory
  mcm.set_walk_vel(h_vel)

  -- Tell the walk engine to update the velocity
  local ret = walk_ch:send('set_velocity')
end

function state.exit()
  print(_NAME..' Exit' ) 
  mcm.set_walk_vel({0,0,0})
  -- Also reset the human input
  hcm.set_motion_velocity({0,0,0})
end

return state