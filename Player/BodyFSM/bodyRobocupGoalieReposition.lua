local state = {}
state._NAME = ...

local Body = require'Body'
local util   = require'util'
local robocupplanner = require'robocupplanner'
local timeout = 10.0
local t_entry, t_update, t_exit

local simple_ipc = require'simple_ipc'
local motion_ch = simple_ipc.new_publisher('MotionFSM!')


local target_pose

function state.entry()
  print(state._NAME..' Entry' )
  -- Update the time of entry
  local t_entry_prev = t_entry -- When entry was previously called
  t_entry = Body.get_time()
  t_update = t_entry
  --hcm.set_ball_approach(0)

  local pose = wcm.get_robot_pose()
  local ballx = wcm.get_ball_x()
  local bally = wcm.get_ball_y()    
  local balla = math.atan2(bally,ballx)
  local ball_local = {ballx,bally,balla}
  local ballGlobal = util.pose_global(ball_local, pose)

  target_pose = robocupplanner.getGoalieTargetPose(pose,ballGlobal)
 
end

function state.update()
  if mcm.get_walk_ismoving()==0 then
    motion_ch:send'hybridwalk'
  end
  --  print(state._NAME..' Update' )
  -- Get the time of update
  local t  = Body.get_time()
  local dt = t - t_update
  -- Save this at the last update time
  t_update = t

  --[[
  if t-t_entry > timeout then
    return'timeout'
  end
--]]
  local pose = wcm.get_robot_pose()
  local move_vel,reached = robocupplanner.getVelocityGoalie(pose,target_pose)
  if not reached then
    mcm.set_walk_vel(move_vel)    
  else
    return 'done'
  end
end

function state.exit()
  print(state._NAME..' Exit' )
  t_exit = Body.get_time()
  mcm.set_walk_stoprequest(1)
end

return state
