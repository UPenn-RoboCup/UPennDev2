local state = {}
state._NAME = ...
local Body   = require'Body'
local util   = require'util'
local vector = require'vector'
local libStep = require'libStep'
-- FSM coordination
local simple_ipc = require'simple_ipc'
local motion_ch = simple_ipc.new_publisher('MotionFSM!')

local robocupplanner = require'robocupplanner'

-- Get the human guided approach
require'hcm'
-- Get the robot guided approach
require'wcm'

require'mcm'



local t_entry, t_update, t_exit
local nwaypoints, wp_id
local waypoints = {}

local target_pose
local uLeft_now, uRight_now, uTorso_now, uLeft_next, uRight_next, uTorso_next
local supportLeg







--[[

local function robocup_follow( pose, target_pose)
  local maxStep = 0.05
  local maxTurn = 0.15
  local dist_threshold = Config.fsm.bodyRobocupFollow.th_dist
  local angle_threshold = .1



   

  -- Angle towards the waypoint
  local aTurn = util.mod_angle(math.atan2(rel_pose[2],rel_pose[1]))

  -- calculate walk step velocity based on ball position
  local vStep = vector.zeros(3)
  -- TODO: Adjust these constants
  vStep[1] = .25 * rel_pose[1]
  vStep[2] = .25 * rel_pose[2]

  -- Reduce speed based on how far away from the waypoint we are
  if rel_dist < 0.1 then maxStep = 0.02 end
  local scale = math.min(maxStep/math.sqrt(vStep[1]^2+vStep[2]^2), 1)
  vStep = scale * vStep
 

  if rel_dist<dist_threshold then
    if math.abs(aTurn)<angle_threshold then
    -- if not the last waypoint, then we are done with this waypoint
      return {0,0,0}, true
    else
      vStep[3] = math.min(
         Config.walk.maxTurnSpeed,
         math.max(-Config.walk.maxTurnSpeed,
         Config.walk.aTurnSpeed * aTurn))
         vStep[1],vStep[2] = 0,0

    end
  else
    vStep[3] = math.min(
       Config.walk.maxTurnSpeed,
       math.max(-Config.walk.maxTurnSpeed,
       Config.walk.aTurnSpeed * aTurn))
  end

  return vStep, false
end

--]]

function state.entry()
  print(state._NAME..' Entry' )
  -- Update the time of entry
  local t_entry_prev = t_entry -- When entry was previously called
  t_entry = Body.get_time()
  t_update = t_entry
  motion_ch:send'hybridwalk'
end

function state.update()
  --print(state._NAME..' Update' )
  -- Get the time of update
  local t  = Body.get_time()
  local dt = t - t_update
  -- Save this at the last update time
  t_update = t

  local ballr, vStep
  local reached = false
  if IS_WEBOTS then
    local pose = wcm.get_robot_pose_gps()
--    print(pose[1],pose[2],pose[3]*180/math.pi)
    local foot_xOffset = 0.15
    local ballx = wcm.get_ball_x() - foot_xOffset
    local bally = wcm.get_ball_y()
    ballr = math.sqrt(ballx*ballx+bally*bally)
    local balla = math.atan2(bally,ballx)
    local walk_target_local = {ballx,bally,balla}
    local ballGlobal = util.pose_global(walk_target_local, pose)

    local target_pose = robocupplanner.getTargetPose(pose,ballGlobal)


--    local vStep = robocup_follow( pose, target_pose)
    vStep,reached = robocupplanner.getVelocity(pose,target_pose)
    mcm.set_walk_vel(vStep)

  else
    local pose = wcm.get_robot_pose()

    local foot_xOffset = 0.15
    local ballx = wcm.get_ball_x() - foot_xOffset
    local bally = wcm.get_ball_y()
    ballr = math.sqrt(ballx*ballx+bally*bally)
    local balla = math.atan2(bally,ballx)
    local walk_target_local = {ballx,bally,balla}

    local target_pose = util.pose_global(walk_target_local, pose)

--    local vStep = robocup_follow( pose, target_pose)
    vStep,reached = robocupplanner.getVelocity(pose,target_pose)
    mcm.set_walk_vel(vStep)
  end

 local ball_elapsed = t - wcm.get_ball_t()
 if ball_elapsed <0.5 and reached then
   return 'ballclose'
 end

end

function state.exit()
  print(state._NAME..' Exit' )
end

return state
