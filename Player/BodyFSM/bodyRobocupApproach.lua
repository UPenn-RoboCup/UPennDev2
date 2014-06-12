local state = {}
state._NAME = ...
local Body   = require'Body'
local util   = require'util'
local vector = require'vector'
local libStep = require'libStep'
-- FSM coordination
local simple_ipc = require'simple_ipc'
local motion_ch = simple_ipc.new_publisher('MotionFSM!')


-- Get the human guided approach
require'hcm'
-- Get the robot guided approach
require'wcm'

require'mcm'


local step_planner
local t_entry, t_update, t_exit
local nwaypoints, wp_id
local waypoints = {}

local target_pose
local uLeft_now, uRight_now, uTorso_now, uLeft_next, uRight_next, uTorso_next
local supportLeg

local ball_side = 1

local function robocup_approach( pose, target_pose)
  local maxStep = 0.05
  local maxTurn = 0.15
  local dist_threshold = Config.fsm.bodyRobocupFollow.th_dist
  local angle_threshold = .1

  -- Distance to the waypoint
  local rel_pose = util.pose_relative(target_pose,pose)
  local rel_dist = math.sqrt(rel_pose[1]*rel_pose[1]+rel_pose[2]*rel_pose[2])

  -- calculate walk step velocity based on ball position
  local vStep = vector.zeros(3)
  -- TODO: Adjust these constants
  vStep[1] = .25 * rel_pose[1]
  vStep[2] = .25 * rel_pose[2]
  vStep[3]=0

  -- Reduce speed based on how far away from the waypoint we are
  if rel_dist < 0.1 then maxStep = 0.02 end
  local scale = math.min(maxStep/math.sqrt(vStep[1]^2+vStep[2]^2), 1)
  vStep = scale * vStep
 
  return vStep, false
end



function state.entry()
  print(state._NAME..' Entry' )
  -- Update the time of entry
  local t_entry_prev = t_entry -- When entry was previously called
  t_entry = Body.get_time()
  t_update = t_entry
  
  local bally = wcm.get_ball_y()
  if bally<0 then
    ball_side = -1
  else
    ball_side = 1
  end
end

function state.update()
  --print(state._NAME..' Update' )
  -- Get the time of update
  local t  = Body.get_time()
  local dt = t - t_update
  -- Save this at the last update time
  t_update = t


  local pose = wcm.get_robot_pose()

  local foot_xOffset = 0.30
  local ballx = wcm.get_ball_x() - foot_xOffset
  local bally = wcm.get_ball_y() - (ball_side*0.12)
  local ballr = math.sqrt(ballx*ballx+bally*bally)
  local balla = math.atan2(bally,ballx)
  local walk_target_local = {ballx,bally,balla}
  local target_pose = util.pose_global(walk_target_local, pose)

  local vStep,arrived = robocup_approach( pose, target_pose)
  mcm.set_walk_vel(vStep)

  if ballr > 1.0 then return 'ballfar' end

 local ball_elapsed = t - wcm.get_ball_t()
 if ball_elapsed <0.5 and 
    ballx<0.10 and
    bally<0.02 and bally > -0.02 then
    print("Ball pos:",wcm.get_ball_x(),wcm.get_ball_y())
    if ball_side==1 then
      mcm.set_walk_kickfoot(0)--left foot kick
    else
      mcm.set_walk_kickfoot(1)--right foot kick
    end
    mcm.set_walk_kicktype(0) --this means real mode (keep run body fsm after kicking)
   return 'done'
 end

end

function state.exit()
  print(state._NAME..' Exit' )
end

return state
