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
  local rel_dist = math.sqrt(rel_wp.x^2 + rel_wp.y^2)
  
  -- Angle towards the waypoint
  local aTurn = util.mod_angle(math.atan2(rel_wp.y,rel_wp.x))

  -- calculate walk step velocity based on ball position
  local vStep = vector.zeros(3)
  -- TODO: Adjust these constants
  vStep[1] = .60 * rel_wp.x
  vStep[2] = .75 * rel_wp.y
  
  -- Reduce speed based on how far away from the waypoint we are
  local scale = math.min(maxStep/math.sqrt(vStep[1]^2+vStep[2]^2), 1)
  vStep = scale * vStep
  vStep[3] = 0.75*aTurn

  -- If we are close to the waypoint and have the right angle threshold, we are fine
  -- TODO: Only with the last point do we care about the angle
  print('Relative distances',rel_dist,rel_wp.a*180/math.pi)
  if rel_dist<dist_threshold then
    -- if not the last waypoint, then we are done with this waypoint
    if wp_id<nwaypoints then return {0,0,0}, true end
    -- else, we DO need to end with the right orientation
    if math.abs(rel_wp.a)<angle_threshold then return {0,0,0}, true end
    -- Just turn in place since we are close
    print('Turning in place')
    vStep[3] = .5*rel_wp.a
  end

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
  mcm.set_walk_vel{0,0,0}

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
  print(state._NAME..' Exit' )
  -- Zero the velocity
  mcm.set_walk_vel{0,0,0}
end

return state