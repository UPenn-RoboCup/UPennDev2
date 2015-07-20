local state = {}
state._NAME = ...

local Body = require'Body'

local timeout = 10.0
local t_entry, t_update, t_exit

local VX_WALK = 0.1
local VY_WALK = 0.1
local VA_WALK = 10*DEG_TO_RAD

-- Ideal position in y along the center
local Y_THRESH = 0.07
local Y_MAX = 1
local Y_FACTOR = 0.7
--
local X_THRESH = 0.07
local X_GOAL = -4.05
--
local A_THRESH = 5 * DEG_TO_RAD
--
local sign = require'util'.sign
local pose_relative = require'util'.pose_relative

local TIMEOUT = 15

function state.entry()
  print(state._NAME..' Entry' )
  -- Update the time of entry
  local t_entry_prev = t_entry -- When entry was previously called
  t_entry = Body.get_time()
  t_update = t_entry
  head_ch:send'line'
end

function state.update()
  if not gcm.get_fsm_Motion():find('HybridWalk') then
    motion_ch:send'hybridwalk'
  end

  --  print(state._NAME..' Update' )
  -- Get the time of update
  local t  = Body.get_time()
  local dt = t - t_update

  -- Save this at the last update time
  t_update = t

  local ball = wcm.get_robot_ballglobal()

  -- If not on our side of the field, then do not move yet
  if ball[1] > -0.1 then
    return 'idle'
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

  local in_position = true
  local vx = 0
  local vy = 0
  local va = 0

  -- Stay in front of the ball always
  if math.abs(dPose.y) > Y_THRESH then
    vy = sign(dPose.y) * VY_WALK
    in_position = false
  end

  -- Angle to face the ball a bit
  if math.abs(dPose.a) > A_THRESH then
    va = sign(dPose.a) * VA_WALK
    in_position = false
  end

  -- We should move up from the goal line
  if math.abs(dPose.x) > X_THRESH then
    vx = sign(dPose.x) * VX_WALK
    if vx > 0 then
      vx = vx * 0.5
    end
    in_position = false
  end

  -- If in position, then return
  if in_position then
    print('GoaliePosition | dPose', dPose, pose)
    return'idle'
  end

  local vel = vector.new{vx, vy, va}

  mcm.set_walk_vel(vel)

  if t-t_entry > TIMEOUT then
    return'timeout'
  end

end

function state.exit()
  print(state._NAME..' Exit' )
  t_exit = Body.get_time()
end

return state
