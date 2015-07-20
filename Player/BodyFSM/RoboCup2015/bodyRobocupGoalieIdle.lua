local state = {}
state._NAME = ...

local Body = require'Body'

local timeout = 10.0
local t_entry, t_update, t_exit

-- Ideal position in y along the center
local Y_THRESH = 0.1
local Y_MAX = 1
local Y_FACTOR = 0.7
--
local X_THRESH = 0.05
local X_GOAL = -4.05
--
local A_THRESH = 10 * DEG_TO_RAD
--
local sign = require'util'.sign
local pose_relative = require'util'.pose_relative

local TIMEOUT = 20

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

  head_ch:send'line'

end

function state.update()

  --  print(state._NAME..' Update' )
  -- Get the time of update
  local t  = Body.get_time()
  local dt = t - t_update

  -- Save this at the last update time
  t_update = t

  local ball = wcm.get_robot_ballglobal()
  -- check ball_t

  -- If not on our side of the field, then do not move yet
  if ball[1] > -0.1 and t - wcm.get_ball_t() < 5 then
    return
  end

  local pose = vector.pose(wcm.get_robot_pose())

  -- Find the optimal pose
  local y_goal = Y_FACTOR * math.min(math.max(-Y_MAX, ball[2]), Y_MAX);
  local a_goal = 0
  local goalPose = vector.pose{
    X_GOAL,
    y_goal,
    a_goal
  }
  local dPose = pose_relative(goalPose, pose)
  --print('dPose', dPose, 'goalPose', goalPose, 'pose', pose)
  --print('ball', unpack(ball))

  local in_position = true

  -- We should move up from the goal line
  if math.abs(dPose.x) > X_THRESH then
    in_position = false
  end

  -- Stay in front of the ball always
  if math.abs(dPose.y) > Y_THRESH then
    in_position = false
  end

  -- Angle to face the ball a bit
  if math.abs(dPose.a) > A_THRESH then
    in_position = false
  end

  -- If in position, then return
  if not in_position then
    print('GoalieIdle | dPose', dPose, pose)
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
