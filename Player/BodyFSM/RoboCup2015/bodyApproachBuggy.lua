local state = {}
state._NAME = ...
local Body   = require'Body'
local util   = require'util'
local vector = require'vector'

local USE_ADJUSTMENT = false

local t_entry, t_update, t_exit
local wp_thread
local dist_threshold = 0.01
local angle_threshold = 2 * DEG_TO_RAD
local maxStep = 0.07
local maxTurn = 0.10
local finished,pose0

local sqrt = math.sqrt
local pow = math.pow
local function pDist(p) return sqrt(pow(p.x,2)+pow(p.y,2)) end


local function robocup_approach(target_pose, pose)
  -- Distance to the waypoint
  local rel_pose = util.pose_relative(target_pose, pose)
  local rel_dist = pDist(rel_pose)

  -- calculate walk step velocity based on ball position
  local vStep = vector.pose{
		util.procFunc(rel_pose.x*0.5, 0, maxStep),
		util.procFunc(rel_pose.y*0.5, 0, maxStep),
		rel_pose.a * 0.5
	}

  -- Reduce speed based on how far away from the waypoint we are
	local maxStep1 = rel_dist < 0.01 and 0.02 or maxStep
  local scale = math.min(maxStep1/pDist(vStep), 1)

  return scale * vStep, rel_dist, math.abs(rel_pose.a)
end

function follower(waypoints)
	local zero_vel = vector.zeros(3)
	local pose = coroutine.yield()
	for i, wp in ipairs(waypoints) do
		print('bodyApproach | Waypoint', wp)
		local dR = math.huge
		local dA = math.huge
		local vel = zero_vel
		while dR>dist_threshold or dA>angle_threshold do
			vel, dR, dA = robocup_approach(wp, pose)
			pose = coroutine.yield(vel)
		end
		pose = coroutine.yield(zero_vel)
	end
	return zero_vel
end


function state.entry()
  print(state._NAME..' Entry' )
  -- Update the time of entry
  local t_entry_prev = t_entry -- When entry was previously called

  t_entry = Body.get_time()
  t_update = t_entry

	-- Check for waypoints
	local waypoints = {vector.pose(hcm.get_teleop_waypoint())}
	pose0=wcm.get_robot_pose()

	-- Make our waypoint follower
	wp_thread = coroutine.create(follower)
	-- set the waypoints
	coroutine.resume(wp_thread, waypoints)

	-- Start walking
	motion_ch:send'hybridwalk'
	finished = false
end

function state.update()
  --print(state._NAME..' Update' )
  local t  = Body.get_time()
  local dt = t - t_update
  -- Save this at the last update time
  t_update = t

	if finished then
    if mcm.get_walk_ismoving()==0 then return "done" end
    return
  end

	-- Get any human given bias in the walk
	local pose = vector.pose(wcm.get_robot_pose())
	local status, velocity = coroutine.resume(wp_thread, pose)
	--print('state._NAME | velocity', status, velocity)

	-- Exit if the coroutine has no more waypoints left to follow
	if not status then
		mcm.set_walk_stoprequest(1)
		finished=true
	end

	-- Set the walking velocity
	mcm.set_walk_vel(velocity)

end

function state.exit()
	local movement = util.pose_relative(wcm.get_robot_pose(),pose0)
  print(string.format("Final movement: %.3f %.3f %.1f",movement[1],movement[2],movement[3]*180/math.pi))

  print(state._NAME..' Exit' )
	mcm.set_walk_vel({0,0,0})
end

return state
