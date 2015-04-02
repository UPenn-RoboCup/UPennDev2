local state = {}
state._NAME = ...
local Body   = require'Body'
local util   = require'util'
local vector = require'vector'
-- FSM coordination
local motion_ch = require'si'.new_publisher('MotionFSM!')
require'hcm'
require'wcm'
require'mcm'


local step_planner
local t_entry, t_update, t_exit
local nwaypoints, wp_id
local waypoints = {}

local function robocup_approach(pose, target_pose)
  --local maxStep = 0.04
  local maxStep = 0.08
  local maxTurn = 0.15
  local dist_threshold = Config.fsm.bodyRobocupFollow.th_dist
  local angle_threshold = .1

  -- Distance to the waypoint
  local rel_pose = util.pose_relative(target_pose,pose)
  local rel_dist = math.sqrt(rel_pose[1]*rel_pose[1]+rel_pose[2]*rel_pose[2])

  -- calculate walk step velocity based on ball position
  local vStep = vector.zeros(3)
  -- TODO: Adjust these constants
 
  vStep[1] = math.min(maxStep,math.max(-maxStep,rel_pose[1]*0.5))
  vStep[2] = math.min(maxStep,math.max(-maxStep,rel_pose[2]*0.5))
  vStep[3]=0

  -- Reduce speed based on how far away from the waypoint we are
  if rel_dist < 0.04 then maxStep = 0.02 end
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

  local ballx = wcm.get_ball_x()
  local bally = wcm.get_ball_y()
  local pose = wcm.get_robot_pose()
  local ballGlobal = util.pose_global({ballx,bally,0},pose)

  last_ph = 0  
  last_step = 0

end

function state.update()
  --print(state._NAME..' Update' )
  local t  = Body.get_time()
  local dt = t - t_update
  -- Save this at the last update time
  t_update = t



end

function state.exit()
  print(state._NAME..' Exit' )
end

return state
