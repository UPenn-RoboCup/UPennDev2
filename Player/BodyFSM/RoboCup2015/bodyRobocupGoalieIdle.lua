local state = {}
state._NAME = ...

local Body = require'Body'

local t_entry, t_update, t_exit

-- Ideal position in y along the center
local X_GOAL = -4
local Y_MAX = 1
local Y_FACTOR = 0.7
--
local X_THRESH = 0.05
local Y_THRESH = 0.1
local A_THRESH = 10 * DEG_TO_RAD
--
local sign = require'util'.sign
local pose_relative = require'util'.pose_relative

local TIMEOUT = 20

local has_spreaded = false

function state.entry()
  print(state._NAME..' Entry' )
  -- Update the time of entry
  local t_entry_prev = t_entry -- When entry was previously called
  t_entry = Body.get_time()
  t_update = t_entry

  print('Goalie Pose', vector.pose(wcm.get_robot_pose()))

  if mcm.get_walk_ismoving()>0 then
    print("requesting stop")
    mcm.set_walk_stoprequest(1)
  end

  wcm.set_ball_backonly(0)
  has_spreaded = false
--  head_ch:send'line'

end

function state.update()

  --  print(state._NAME..' Update' )
  -- Get the time of update
  local t  = Body.get_time()
  local dt = t - t_update


  if not has_spreaded and 
    mcm.get_walk_ismoving()==0 and
    Config.goalie_spread_enable then
    has_spreaded=true
    mcm.set_walk_kicktype(9)
    mcm.set_walk_kickfoot(1)--left foot kick
    mcm.set_walk_steprequest(1)
    mcm.set_walk_kickphase(1)    
  end


  -- Save this at the last update time
  t_update = t

  local ball = wcm.get_robot_ballglobal()
  -- check ball_t

  --ball should be close and recently seen
  if ball[1] > -1 or t - wcm.get_ball_t() > .2 then
    return
  end

  local pose = vector.pose(wcm.get_robot_pose())

  -- Find the optimal pose
  local y_goal = Y_FACTOR * math.min(math.max(-Y_MAX, ball[2]), Y_MAX);
  local a_goal = 0
  local goalPose = vector.pose{X_GOAL,y_goal,a_goal}
  local dPose = pose_relative(goalPose, pose)
  --print('dPose', dPose, 'goalPose', goalPose, 'pose', pose)
  --print('ball', unpack(ball))

  -- We should move up from the goal line
  if
--math.abs(dPose.x) > X_THRESH or
math.abs(dPose.y) > Y_THRESH then
    print("Ball Found:",ball[1],ball[2])
    print('GoalieIdle | goalPose', goalPose[1],goalPose[2])
    motion_ch:send'hybridwalk' --start moving
    return'position'
  end

  if t-t_entry > TIMEOUT then
    return'timeout'
  end
end

function state.exit()
  print(state._NAME..' Exit' )
  t_exit = Body.get_time()
end

return state
