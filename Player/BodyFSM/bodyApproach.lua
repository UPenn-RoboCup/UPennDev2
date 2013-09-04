local state = {}
state._NAME = ...
local Body  = require'Body'

-- Get the human guided approach
require'hcm'
-- Get the robot guided approach
-- wcm is needed for pose, too
require'wcm'

local t_entry, t_update, t_exit
local nwaypoints, wp_id
local waypoints = {}


local dist_threshold  = 0.02
local angle_threshold = 1*math.pi/180
local turn_threshold  = 5*math.pi/180

local function pose_idx(t,idx)
  if idx=='x' then
    return t[1]
  elseif idx=='y' then
    return t[2]
  elseif idx=='a' then
    return t[3]
  end
end

local function pose_newidx(t,idx)
  if idx=='x' then
    return t[1]
  elseif idx=='y' then
    return t[2]
  elseif idx=='a' then
    return t[3]
  end
end

function state.entry()
  print(state._NAME..' Entry' )
  -- Update the time of entry
  local t_entry_prev = t_entry -- When entry was previously called
  t_entry = Body.get_time()
  t_update = t_entry
  -- Zero the velocity
  -- Initially stop movement
  mcm.set_walk_vel{0,0,0}
  hcm.set_motion_velocity{0,0,0}
  
  -- Grab the pose
  local pose = wcm.get_robot_pose()

  -- Grab the waypoints
  nwaypoints = hcm.get_motion_nwaypoints()
  local raw_waypoints = vector.slice(hcm.get_motion_waypoints(),3*nwaypoints)

  -- Check the frame of reference
  local waypoint_frame = hcm.get_motion_waypoint_frame()

  local idx = 1
  for w=1,nwaypoints do
    local waypoint = vector.slice(raw_waypoints,idx,idx+2)
    if waypoint_frame==0 then
      -- If a relative frame, then convert to the global frame
      waypoint = util.pose_global(waypoint, pose)
    end
    -- Add waypoint.x, waypoint.y, waypoint.a
    local mt = getmetatable(waypoint)
    mt.__index = pose_idx
    -- Add to the waypoints table
    waypoints[w] = setmetatable(waypoint,mt)
    -- Increment the index
    idx = idx + 3
  end

  -- Start with the first waypoint
  wp_id = 1
  
end

function state.update()
  -- print(state._NAME..' Update' ) 
  -- Get the time of update
  local t  = Body.get_time()
  local dt = t - t_update
  -- Save this at the last update time
  t_update = t

  -- Grab the current waypoint
  local wp = waypoints[wp_id]

  -- Grab the current pose
  local pose = wcm.get_robot_pose()

  -- Set with relative coordinates
  rel_wp = util.pose_relative(wp,pose)

  -- Add waypoint.x, waypoint.y, waypoint.a
  local mt = getmetatable(rel_wp)
  mt.__index = pose_idx
  -- Add to the waypoints table
  rel_wp = setmetatable(rel_wp,mt)

  -- Distance to the waypoint
  local wpR = math.sqrt(rel_wp.x^2 + rel_wp.y^2)
  local aTurn = util.mod_angle(math.atan2(rel_wp.y,rel_wp.x))

  -- Find the walk velocity
  
  --[[
  print("Current pose:",pose[1],pose[2],pose[3]*180/math.pi)
  print("Current target:",target_pose[1],target_pose[2] )
  print("Rel pose:",relPoseTarget[1],relPoseTarget[2] )
  print("target angle:",aTurn/math.pi*180)
  print("\n")
  --]]

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