local state = {}
state._NAME = ...
local Body   = require'Body'
local util   = require'util'
local vector = require'vector'

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
local maxStep = .1

-- Thresholds for moving to the next waypoint
local dist_threshold  = 0.15
local angle_threshold = 2*math.pi/180
-- Thresholds for spinning to face the waypoint
local spin_threshold = 5*math.pi/180

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
  vStep[3] = util.procFunc(0.75*aTurn,0,.5)

  -- If we are close to the waypoint and have the right angle threshold, we are fine
  -- TODO: Only with the last point do we care about the angle
  --print('Relative distances',rel_dist,rel_wp.a*180/math.pi)
  if rel_dist<dist_threshold then
    -- if not the last waypoint, then we are done with this waypoint
    if wp_id<nwaypoints then return {0,0,0}, true end
    -- else, we DO need to end with the right orientation
    if math.abs(rel_wp.a)<angle_threshold then return {0,0,0}, true end
    -- Just turn in place since we are close
    --print('Turning in place')
    vStep[3] = .05*util.sign(rel_wp.a)
  end

  return vStep, false

end

local function simple_follow(pose,wp,rel_wp)

  -- Distance to the waypoint
  local rel_dist = math.sqrt(rel_wp.x^2 + rel_wp.y^2)
  
  -- Angle towards the waypoint
  local aTurn = util.mod_angle(math.atan2(rel_wp.y,rel_wp.x))
  --[[
  local aTurn = 0
  if rel_wp.x<-dist_threshold then
    print('backup!')
  else
    aTurn = util.mod_angle(math.atan2(rel_wp.y,rel_wp.x))
  end
  --]]

  -- calculate walk step velocity based on ball position
  local vStep = vector.zeros(3)
  -- Go straight to the waypoint
  vStep[1] = .6 * rel_wp.x
  vStep[1] = math.min(maxStep/math.abs(vStep[1]), 1) * vStep[1]
  --[[
  if rel_wp.x<-dist_threshold then
    vStep[1] = -maxStep
  end
  --]]
  -- Spin to face the waypoint
  vStep[3] = util.procFunc(0.75*aTurn,0,.1)
  if math.abs(aTurn)>spin_threshold and rel_wp.x>0 then
    -- Spin in place to face the waypoint
    vStep[1] = 0
  end

  -- If we are close to the waypoint and have the right angle threshold
  -- Then we are done
  if rel_dist<dist_threshold then
    --print('dist in thresh')
    -- if not the last waypoint, then we are done with this waypoint
    if wp_id<nwaypoints then return {0,0,0}, true end
    -- else, we DO need to end with the right orientation
    if math.abs(rel_wp.a)<angle_threshold then return {0,0,0}, true end
    vStep[3] = .05*util.sign(rel_wp.a)
  end

  return vStep, false
end

local follow = {
  [1] = robocup_follow,
  [2] = simple_follow,
}

function state.entry()
  print(state._NAME..' Entry' )
  -- Update the time of entry
  local t_entry_prev = t_entry -- When entry was previously called
  t_entry = Body.get_time()
  t_update = t_entry
  
  -- Grab the pose
  --local pose = wcm.get_robot_pose()
  local pose = wcm.get_slam_pose()

  -- Grab the waypoints
  nwaypoints = hcm.get_motion_nwaypoints()
  print('# of waypoints:', nwaypoints)
  print('waypoints', unpack(hcm.get_motion_waypoints()))
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
    -- Add to the waypoints table
    waypoints[w] = waypoint
    -- Increment the index
    idx = idx + 3
  end

  -- Start with the first waypoint
  wp_id = 1

  -- Zero the velocity
  mcm.set_walk_vel({0,0,0})

  -- Begin walking
  motion_ch:send'walk'
  
end

local cnt = 0
function state.update()
  --print(state._NAME..' Update' ) 
  -- Get the time of update
  local t  = Body.get_time()
  local dt = t - t_update
  -- Save this at the last update time
  t_update = t

  -- If ID out of bounds
  --print('wp_id',wp_id,'of',nwaypoints)
  if wp_id>nwaypoints then return'done' end
  if wp_id<1 then return'error' end

  -- Grab the current waypoint
  local wp = waypoints[wp_id]

  -- Grab the current pose
  cnt = cnt + 1
  --local pose = vector.pose(wcm.get_robot_pose())
  local pose = vector.pose(wcm.get_slam_pose())
  if cnt % 20 == 0 then
    print('Robot pose:', unpack(pose) )
  end

  -- Set with relative coordinates
  local rel_wp = util.pose_relative(wp,pose)

  -- Debug
  --[[
  print('pose',pose)
  --print('wayp',wp)
  print('rela',rel_wp)
  --]]

  -- Grab the follow mode
  local mode = hcm.get_motion_follow_mode()
  local up = follow[mode]
  if not up then return'error' end
  local vel, at_waypoint = up(pose,wp,rel_wp)

  -- Update the velocity
  mcm.set_walk_vel(vel)

  -- Check if we are at the waypoint
  if at_waypoint then
    wp_id = wp_id + 1
  end

end

function state.exit()
  print(state._NAME..' Exit' )
  -- Zero the velocity
  mcm.set_walk_vel{0,0,0}
end

return state
