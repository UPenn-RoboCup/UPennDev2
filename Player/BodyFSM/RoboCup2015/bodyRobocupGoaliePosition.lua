local state = {}
state._NAME = ...

local Body = require'Body'

local timeout = 10.0
local t_entry, t_update, t_exit

local VX_WALK = 0.1
local VY_WALK = 0.1
local VA_WALK = 10*DEG_TO_RAD

-- Ideal position in y along the center
local Y_THRESH = 0.10
--
local X_THRESH = 0.10
local X_GOAL = -4.25
--
local A_THRESH = 5 * DEG_TO_RAD
--
local sign = require'util'.sign
local pose_relative = require'util'.pose_relative

function state.entry()
  print(state._NAME..' Entry' )
  -- Update the time of entry
  local t_entry_prev = t_entry -- When entry was previously called
  t_entry = Body.get_time()
  t_update = t_entry

  motion_ch:send'hybridwalk'
end

function state.update()

  --  print(state._NAME..' Update' )
  -- Get the time of update
  local t  = Body.get_time()
  local dt = t - t_update

  -- Save this at the last update time
  t_update = t

  local ball = wcm.get_robot_ballglobal()
  --print('ball*', ball)
  local pose = vector.pose(wcm.get_robot_pose())

  -- Find the optimal pose
  local y_goal = math.min(math.max(-1, ball[2]), 1);
  local goalPose = vector.pose{
    X_GOAL,
    y_goal,
    math.atan2(y_goal, 1)
  }
  --print('goalPose', goalPose)
  local dPose = pose_relative(goalPose, pose)

  local in_position = true
  local vx = 0
  local vy = 0
  local va = 0

  -- We should move up from the goal line
  if math.abs(dPose.x) > X_THRESH then
    vx = sign(dPose.x) * VX_WALK
    in_position = false
  end

  -- Stay in front of the ball always
  if math.abs(dPose.y) > Y_THRESH then
    vy = sign(dPose.y) * VY_WALK
    in_position = false
  end

  -- Angle to face the ball a bit
  --local da = math.atan2(goalPose.y, 0)

  if math.abs(dPose.a) > A_THRESH then

    --if vx==0 and vy==0 then
      --print('dPose.a', dPose.a)
      --print('goalPose.y', goalPose.y)
      va = sign(dPose.a) * VA_WALK
      in_position = false
    --end
  end

  -- If in position, then return
  if in_position then
    return'idle'
  end

  local vel = vector.new{vx, vy, va}
  --local diff = vector.new{dx, dy, da}

  --[[
  print('ball', ball)
  print('goalPose', goalPose)
  print('pose', pose)
  print('diff', diff)
  print('vel', vel)
  --]]
  mcm.set_walk_vel(vel)

end

function state.exit()
  print(state._NAME..' Exit' )
  t_exit = Body.get_time()
end

return state
