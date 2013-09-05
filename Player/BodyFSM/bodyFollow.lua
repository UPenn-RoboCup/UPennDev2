local state = {}
state._NAME = ...
local Body  = require'Body'
local util  = require'util'

-- Get the human guided approach
require'hcm'
-- Get the robot guided approach
-- wcm is needed for pose, too
require'wcm'

-- FSM coordination
local simple_ipc = require'simple_ipc'
local motion_ch = simple_ipc.new_publisher('MotionFSM',true)

local t_entry, t_update, t_exit
local nwaypoints, wp_id
local waypoints = {}

-- maximum stride length to take
local maxStep = .05

-- Thresholds for moving to the next waypoint
local dist_threshold  = 0.02
local angle_threshold = 1*math.pi/180
local turn_threshold  = 5*math.pi/180

local function robocup_follow( pose, wp, rel_wp )

  -- Distance to the waypoint
  local wpR = math.sqrt(rel_wp.x^2 + rel_wp.y^2)
  -- Angle to the waypoint
  local aTurn = util.mod_angle(math.atan2(rel_wp.y,rel_wp.x))
  print('dist',wpR,aTurn)

  if wpR<dist_threshold and math.abs(aTurn)<angle_threshold then
    return {0,0,0}, true
  end

  -- calculate walk step velocity based on ball position
  local vStep = vector.zeros(3)
  -- TODO: Adjust these constants
  vStep[1] = .60 * (wp.x - pose.x)
  vStep[2] = .75 * (wp.y - pose.y)
  
  -- Reduce speed based on how far away from the waypoint we are
  local scale = math.min(maxStep/math.sqrt(vStep[1]^2+vStep[2]^2), 1)
  vStep = scale * vStep
  --vStep[3] = 0.75*aTurn
  
  return vStep, false

end

local function chase_follow(pose,wp,rel_wp)
  -- Check how close we are to the waypoint
  -- Distance to the waypoint
  local wpR = math.sqrt(rel_wp.x^2 + rel_wp.y^2)
  -- Angle to the waypoint
  local aTurn = util.mod_angle(math.atan2(rel_wp.y,rel_wp.x))

  -- If we are close enough, then increment the waypoint id
  if wpR<dist_threshold then

  end
end

local follow = {
  [1] = robocup_follow,
  [2] = simple_follow,
  [3] = chase_follow,
}

-- Metatables for pose vectors
-- TODO: Use as a utility pose file, too
local function pose_index(p,idx)
  if idx=='x' then
    return p[1]
  elseif idx=='y' then
    return p[2]
  elseif idx=='a' then
    return p[3]
  end
end

local function pose_newindex(p,idx,val)
  if idx=='x' then
    p[1] = val
  elseif idx=='y' then
    p[2] = val
  elseif idx=='a' then
    p[3] = val
  end
end

local function pose_tostring(p)
  return string.format(
    "{x=%g, y=%g, a=%g degrees}",
    p[1], p[2], p[3]*180/math.pi
  )
end

function state.entry()
  print(state._NAME..' Entry' )
  -- Update the time of entry
  local t_entry_prev = t_entry -- When entry was previously called
  t_entry = Body.get_time()
  t_update = t_entry
  
  -- Grab the pose
  local pose = wcm.get_robot_pose()

  -- Grab the waypoints
  nwaypoints = hcm.get_motion_nwaypoints()
  local raw_waypoints = vector.slice(hcm.get_motion_waypoints(),1,3*nwaypoints)

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
    mt.__index    = pose_index
    mt.__newindex = pose_newindex
    mt.__tostring = pose_tostring
    -- Add to the waypoints table
    waypoints[w] = setmetatable(waypoint,mt)
    -- Increment the index
    idx = idx + 3
  end

  -- Start with the first waypoint
  wp_id = 1

  -- Zero the velocity
  -- Initially stop movement
  mcm.set_walk_vel{0,0,0}
  hcm.set_motion_velocity{0,0,0}

  -- Begin walking
  motion_ch:send'walk'
  
end

function state.update()
  print()
  print(state._NAME..' Update' ) 
  -- Get the time of update
  local t  = Body.get_time()
  local dt = t - t_update
  -- Save this at the last update time
  t_update = t

  -- If ID out of bounds
  print('wp_id',wp_id,'of',nwaypoints)
  if wp_id>nwaypoints then return'done' end
  if wp_id<1 then return'error' end

  -- Grab the current waypoint
  local wp = waypoints[wp_id]

  -- Grab the current pose
  local pose = wcm.get_robot_pose()

  -- Set with relative coordinates
  local rel_wp = util.pose_relative(wp,pose)

  -- Add pose.x, pose.y, pose.a
  local mt = getmetatable(pose)
  mt.__index    = pose_index
  mt.__newindex = pose_newindex
  mt.__tostring = pose_tostring
  -- Add to the waypoints table
  pose = setmetatable(pose,mt)

  -- Add waypoint.x, waypoint.y, waypoint.a
  local mt = getmetatable(rel_wp)
  mt.__index    = pose_index
  mt.__newindex = pose_newindex
  mt.__tostring = pose_tostring
  -- Add to the waypoints table
  rel_wp = setmetatable(rel_wp,mt)

  -- Debug
  print('pose',pose)
  print('wayp',wp)
  print('rela',rel_wp)

  -- Grab the follow mode
  local mode = hcm.get_motion_follow_mode()
  local up = follow[mode]
  if not up then return'error' end
  local vel, at_waypoint = up(pose,wp,rel_wp)

  -- Update the velocity
  print(vel,at_waypoint)
  mcm.set_walk_vel(vel)
  --mcm.set_walk_vel{0,0,0}

  -- Check if we are at the waypoint
  if at_waypoint then
    wp_id = wp_id + 1
  end

  --[[

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
  --]]

end

function state.exit()
  Body.set_lwheel_velocity(0)
  Body.set_rwheel_velocity(0)
  print(_NAME..' Exit' ) 
end

return state