local state = {}
state._NAME = ...
local Body  = require'Body'

-- Get the human guided approach
require'hcm'
-- Get the robot guided approach
-- wcm is needed for pose, too
require'wcm'

local t_entry, t_update, t_exit
local waypoints, nwaypoints

local dist_threshold  = 0.02
local angle_threshold = 1*math.pi/180
local turn_threshold  = 5*math.pi/180

function state.entry()
  print(_NAME..' Entry' )
  -- Zero the velocity
  -- Initially stop movement
  mcm.set_walk_vel{0,0,0}
  hcm.set_motion_velocity{0,0,0}
  
  -- Grab the pose
  local pose = wcm.get_robot_pose()

  -- Grab the waypoints
  nwaypoints = hcm.get_motion_nwaypoints()
  waypoints = vector.slice(hcm.get_motion_waypoints(),3*nwaypoints)

  -- Check the frame of reference
  local waypoint_frame = hcm.get_motion_waypoint_frame()
  if waypoint_frame==0 then
    -- If a relative frame, then set our local waypoint copy
    -- to a global frame
    local waypoint_idx = 1
    for w=1,nwaypoints do
      local rel_waypoint = vector.slice(waypoints,waypoint_idx,waypoint_idx+2)
      local global_waypoint = util.pose_global(rel_waypoint, pose)
      -- Update our local waypoints array
      waypoints[waypoint_idx] = global_waypoint[1]
      waypoints[waypoint_idx+1] = global_waypoint[2]
      waypoints[waypoint_idx+2] = global_waypoint[3]
      -- Increment the index
      waypoint_idx = waypoint_idx + 3
    end
  end

  waypoint_count = 1
  target_pose = waypoints[waypoint_count]
  target_pose[3]=0
  relPoseTarget = util.pose_relative(target_pose,pose)
  aTurn = util.mod_angle(math.atan2(relPoseTarget[2],relPoseTarget[1]))
  
  print("Current pose:",pose[1],pose[2],pose[3]*180/math.pi)
  print("Current target:",target_pose[1],target_pose[2] )
  print("Rel pose:",relPoseTarget[1],relPoseTarget[2] )
  print("target angle:",aTurn/math.pi*180)
  print("\n")
end

function state.update()
--  print(_NAME..' Update' ) 
  if waypoint_count>#waypoints then return "done" end

  pose = scm:get_pose()
  target_pose = waypoints[waypoint_count]
  target_pose[3]=0

  relPoseTarget = util.pose_relative(target_pose,pose)
  aTurn = util.mod_angle(math.atan2(relPoseTarget[2],relPoseTarget[1]))

  dist_error = math.sqrt(
		(pose[1]-target_pose[1])^2 + 
		(pose[2]-target_pose[2])^2 ) 

  run_reverse = 1
  if math.abs(aTurn)>math.pi/180 * 90  then
    aTurnDir = util.mod_angle(aTurn+math.pi)
    run_reverse = -1
  else
    aTurnDir = aTurn
  end

  if dist_error>dist_threshold then
    forwardVel = math.min(2000, dist_error/0.20 * 2000) + 1000  
    turnVel = math.min(1000, math.abs(aTurnDir)/(10*Body.DEG_TO_RAD)*1000) + 500           
    if aTurnDir<0 then turnVel = -turnVel end
    aTurnMag = math.min(1, math.abs(aTurnDir)/turn_threshold)
    lVel = run_reverse*forwardVel * (1-aTurnMag) - turnVel * aTurnMag
    rVel = run_reverse*forwardVel * (1-aTurnMag) + turnVel * aTurnMag
    Body.set_wheel_velocity_direct({lVel,rVel})
  else
    --waypoint reached, increment counter
    print(string.format("Waypoint %d reached",waypoint_count ))
    waypoint_count = waypoint_count + 1
    forwardVel = 0
  end
end

function state.exit()
  Body.set_lwheel_velocity(0)
  Body.set_rwheel_velocity(0)
  print(_NAME..' Exit' ) 
end

return state