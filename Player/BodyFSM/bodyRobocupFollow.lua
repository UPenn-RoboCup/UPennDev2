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
local last_ph = 0

function state.entry()
  print(state._NAME..' Entry' )
  -- Update the time of entry
  local t_entry_prev = t_entry -- When entry was previously called
  t_entry = Body.get_time()
  t_update = t_entry
  motion_ch:send'hybridwalk'
  last_ph = 0
end

local function update_velocity()
  local pose = wcm.get_robot_pose()
  if IS_WEBOTS and Config.use_gps_pose then pose = wcm.get_robot_pose_gps() end

  local ballx = wcm.get_ball_x()
  local bally = wcm.get_ball_y()  
  local balla = math.atan2(bally,ballx)

  local walk_target_local = {ballx,bally,balla}
  local ballGlobal = util.pose_global(walk_target_local, pose)
  local target_pose = robocupplanner.getTargetPose(pose,ballGlobal)    
  local vStep,reached = robocupplanner.getVelocity(pose,target_pose)
  mcm.set_walk_vel(vStep)

  
  return reached
end



function state.update()
  --print(state._NAME..' Update' )
  -- Get the time of update
  local t  = Body.get_time()
  local dt = t - t_update
  local reached=false
  -- Save this at the last update time
  t_update = t
  local check_ph = 0.95

  local ph = mcm.get_status_ph()
  if last_ph<check_ph and ph>=check_ph then reached=update_velocity() end
  last_ph = ph
  local ball_elapsed = t - wcm.get_ball_t()
  if ball_elapsed <0.5 and reached then return 'ballclose' end
end

function state.exit()
  print(state._NAME..' Exit' )
end

return state
