local state = {}
state._NAME = ...

local Body = require'Body'
local util   = require'util'
local robocupplanner = require'robocupplanner'

local timeout = 10.0
local t_entry, t_update, t_exit

function state.entry()
  print(state._NAME..' Entry' )
  -- Update the time of entry
  local t_entry_prev = t_entry -- When entry was previously called
  t_entry = Body.get_time()
  t_update = t_entry
  --hcm.set_ball_approach(0)
end

function state.update()
  if gcm.get_game_state()~=3 then return'stop' end
  --  print(state._NAME..' Update' )
  -- Get the time of update
  local t  = Body.get_time()
  local dt = t - t_update
  -- Save this at the last update time
  t_update = t

  -- if we see ball right now and ball is far away start moving
  local ball_elapsed = t - wcm.get_ball_t()
  if ball_elapsed < 0.1 then --ball found
    local pose = wcm.get_robot_pose()
    local ballx = wcm.get_ball_x()
    local bally = wcm.get_ball_y()    
    local balla = math.atan2(bally,ballx)
    local ball_local = {ballx,bally,balla}
    local ballGlobal = util.pose_global(ball_local, pose)
  
    local target_pose = robocupplanner.getGoalieTargetPose(pose,ballGlobal)
    --our goal should be always at (-4.5,0,0)
 
    local move_vel,reached = robocupplanner.getVelocityGoalie(pose,target_pose,0.3)

    if not reached then
      print("Current pose:",pose[1],pose[2])
      print("Move target:",target_pose[1],target_pose[2])      
      return 'reposition'
    end
  end

end

function state.exit()
  print(state._NAME..' Exit' )
  t_exit = Body.get_time()
end

return state
