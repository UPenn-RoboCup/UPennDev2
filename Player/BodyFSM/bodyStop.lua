local state = {}
state._NAME = ...
local util   = require'util'


local Body = require'Body'
local robocupplanner = require'robocupplanner'
local timeout = 10.0
local t_entry, t_update, t_exit, t_plan
require'gcm'

--Tele-op state for testing various stuff
--Don't do anything until commanded
local old_state

function state.entry()
  print(state._NAME..' Entry' )
  -- Update the time of entry
  local t_entry_prev = t_entry -- When entry was previously called
  t_entry = Body.get_time()
  t_update = t_entry    
  t_plan = t_entry
  old_state = gcm.get_game_state()
end

local function plan_whole()

  local pose = wcm.get_robot_pose()
  local ballx = wcm.get_ball_x()
  local bally = wcm.get_ball_y()  
  local balla = math.atan2(bally,ballx)
  local walk_target_local = {ballx,bally,balla}
  local ballGlobal = util.pose_global(walk_target_local, pose)

  wcm.set_robot_ballglobal({ballGlobal[1],ballGlobal[2]})

  local max_plan_step = 100
  local count = 0
  local reached = false
  local v

  local xtrail=wcm.get_robot_trajx()
  local ytrail=wcm.get_robot_trajy()
  local lpose={pose[1],pose[2],pose[3]}
  while (not reached) and (count<max_plan_step) do
    local target_pose,rotate = robocupplanner.getTargetPose(lpose,{ballGlobal[1],ballGlobal[2],0})    
    v,reached = robocupplanner.getVelocity(lpose,target_pose,rotate)
    lpose = util.pose_global({v[1],v[2],v[3]} , {lpose[1],lpose[2],lpose[3]})
    count = count+1
    xtrail[count] = lpose[1]
    ytrail[count] = lpose[2]
    if rotate~=0 then reached = false end
  end
  if Config.debug.follow then
    print("Plan steps: ",count)
  end

  wcm.set_robot_traj_num(count)
  wcm.set_robot_trajx(xtrail)
  wcm.set_robot_trajy(ytrail)
end


function state.update()
  --  print(state._NAME..' Update' )
  -- Get the time of update
  local t  = Body.get_time()
  local dt = t - t_update
  -- Save this at the last update time
  t_update = t  


  if t-t_plan>1 then
    t_plan = t

    if gcm.get_game_role()~=2 and 
      gcm.get_game_state()~=3 and 
      mcm.get_motion_state()==4 then
       mcm.set_walk_stoprequest(1) --stop if we're in walk state
    end

    --just force stop the robot in the testing mode
    if gcm.get_game_role()==2 and mcm.get_motion_state()==4 then
      mcm.set_walk_stoprequest(1)
    end

--[[
    if Config.auto_state_advance and gcm.get_game_role()~= 2 then
      if gcm.get_game_state()<3 then
        gcm.set_game_state(gcm.get_game_state()+1)
      elseif gcm.get_game_state()==5 then 
        gcm.set_game_state(0)
      end
    end
    if gcm.get_game_role()==1 and gcm.get_game_state()>0 then
--      plan_whole()
    end
--]]

  end

  if gcm.get_game_state()<3 then
    --Reset pose at initial, ready and set states
    wcm.set_robot_reset_pose(1) 
    wcm.set_robot_timestarted(0)
  end 

  if gcm.get_game_state()==3 then
    if wcm.get_robot_timestarted()==0 then 
      wcm.set_robot_timestarted(t)
      print("COUNTING STARTED!!!")
    end
    if gcm.get_game_role()==0 then --goalie
      print("Goalie start!")
      return'goalie'
    elseif gcm.get_game_role()==1 then
      print("Attacker start!")
      return'play'
    elseif gcm.get_game_role()==2 then
      --Tester does nothing
    end
    wcm.set_robot_timestarted(0)    
  end
end

function state.exit()
  print(state._NAME..' Exit' )
  t_exit = Body.get_time()  
end

return state
