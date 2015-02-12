local state = {}
state._NAME = ...

local Body = require'Body'
local simple_ipc = require'simple_ipc'
--local vector=require'vector'
--local util = require'util'
--local timeout = 10.0
local t_entry, t_update, t_exit

-- Require all necessary fsm channels
local arm_ch    = simple_ipc.new_publisher('ArmFSM!')
local head_ch   = simple_ipc.new_publisher('HeadFSM!')
local lidar_ch  = simple_ipc.new_publisher('LidarFSM!')
local motion_ch = simple_ipc.new_publisher('MotionFSM!')
require'wcm'

function state.entry()
  print(state._NAME..' Entry' )
  --Reset pose
  wcm.set_robot_odometry({0,0,0})
  wcm.set_robot_pose({0,0,0})
  --Reset ball pose
  wcm.set_ball_x(0)
  wcm.set_ball_y(0)
  -- Reset obstacle pos
	wcm.set_obstacle_reset(1)
  
  -- Update the time of entry
  local t_entry_prev = t_entry -- When entry was previously called
  t_entry = Body.get_time()
  t_update = t_entry
  
  -- Torque on the motors...
  Body.set_larm_torque_enable(1)
  Body.set_rarm_torque_enable(1)

  arm_ch:send'init'
  motion_ch:send'stand'
  --lidar_ch:send'pansingle'
  lidar_ch:send'pan'

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

  print(state._NAME..' Exit' )
  t_exit = Body.get_time()
end

return state
