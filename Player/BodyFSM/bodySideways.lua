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
local spin_threshold = 2*math.pi/180


local function turn_in_place(pose,wp,rel_wp)
  if rel_dist<dist_threshold then
    --print('dist in thresh')
    -- if not the last waypoint, then we are done with this waypoint
    if wp_id<nwaypoints then return {0,0,0}, true end
    -- else, we DO need to end with the right orientation
    if math.abs(rel_wp.a)<angle_threshold then return {0,0,0}, true end
    vStep[3] = .05*util.sign(rel_wp.a)
  end

	if math.abs(rel_wp.a) < spin_threshold then
		retun {0,0,0}, true
	end
	
	-- TODO: may need vx, vy to deal with offsets in x, y when turning
	vStep[1], vStep[2] = 0, 0
	-- We only let the robot to turn CLOCKWISE for this task
	vStep[3] = math.min( .05*util.sign(rel_wp.a), 0.25*rel_wp.a )
  return vStep, false
end


function state.entry()
  print(state._NAME..' Entry' )
  -- Update the time of entry
  local t_entry_prev = t_entry -- When entry was previously called
  t_entry = Body.get_time()
  t_update = t_entry
  
  -- Grab the pose
  local pose = wcm.get_slam_pose()

  -- Grab the angles to turn in place
	-- 3 ways: 1) Default: 90 degree, 2) hcm? 3) COST FUNCTION?
	dYaw = 90*Body.DEG_TO_RAD;
	
	-- Target pose (waypoint)
	targetPose = pose
	targetPose[3] = targetPose[3] + dYaw
	-- Leave more space for arms
	-- targetPose[2] = targetPose[2] - 0.1
	
	print('Angle to turn:', dYaw)
	
  -- Zero the velocity
	phase = 1
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

	if phase == 1 then
		-- Grab target pose
	  if not targetPose then return'error' end

	  -- Grab the current pose
	  cnt = cnt + 1
	  --local pose = vector.pose(wcm.get_robot_pose())
	  local pose = vector.pose(wcm.get_slam_pose())
	  if cnt % 20 == 0 then
	    print('Robot pose:', unpack(pose) )
	  end

	  -- Set with relative coordinates
	  local rel_wp = util.pose_relative(targetPose,pose)

	  -- Debug
	  ---[[
	  print('pose',pose)
	  --print('wayp',wp)
	  print('rela',rel_wp)
	  --]]

	  -- Turn in place
		-- Use step rather than walk might be better?
	  local vel, at_waypoint = turn_in_place(pose,wp,rel_wp)

	  -- Update the velocity
	  mcm.set_walk_vel(vel)

	  -- Check if we are at the waypoint
	  if at_waypoint then
	    phase = phase + 1
	  end
	
	elseif phase == 2 then
		-- 1. just move for a fixed period of time blindly?
		-- 2. hcm so that human decides when to stop?
		-- 3. Check slam pose?
		
		-- Use dumb 1 just for testing
		t_start = unix.time()
		if unix.time - t_start < 5 then
			mcm.set_walk_vel({0, -0.02, 0})
		else
			mcm.set_wal_vel({0,0,0})
			phase = phase + 1
		end
		
	elseif phase == 3 then
		-- TODO:Turn back to face the forward direction
			return 'done'
	end

end

function state.exit()
  print(state._NAME..' Exit' )
  -- Zero the velocity
  mcm.set_walk_vel{0,0,0}
end

return state
