local state = {}
state._NAME = ...
local Body   = require'Body'
local util   = require'util'
local vector = require'vector'

local USE_ADJUSTMENT = false

local t_entry, t_update, t_exit
local wp_thread

local dist_threshold = 0.05
local angle_threshold = 5 * DEG_TO_RAD
local maxStep = 0.08
local maxTurn = 0.15

local sqrt = math.sqrt
local pow = math.pow
local function pDist(p)
	return sqrt(pow(p.x,2)+pow(p.y,2))
end

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
	local maxStep1 = rel_dist < 0.04 and 0.02 or maxStep
  local scale = math.min(maxStep1/pDist(vStep), 1)

  return scale * vStep, rel_dist, math.abs(rel_pose.a)
end

function state.entry()
  print(state._NAME..' Entry' )
  -- Update the time of entry
  local t_entry_prev = t_entry -- When entry was previously called

  t_entry = Body.get_time()
  t_update = t_entry

	-- Make our coroutine
	wp_thread = coroutine.create(function(waypoints)
			local pose, pBias = coroutine.yield()
			for i, wp in ipairs(waypoints) do
				io.write('bodyApproach | Waypoint', wp, '\n')
				local betweenWP = true
				while betweenWP do
					local pOffset = util.pose_global(pBias, {0,0,pose[3]})
					local wp_adjusted = util.pose_relative(pOffset, wp)
					local vel, dR, dA = robocup_approach(USE_ADJUSTMENT and wp_adjusted or wp, pose)
					if dR<dist_threshold and dA<angle_threshold then
						betweenWP = false
						vel = {0,0,0}
					end
					pose, pBias = coroutine.yield(vel)
				end
			end
			return {0,0,0}
		end)

	local waypoints = {}
	-- If a demo, then just use those waypoints
	if Config.demo then
		local wps_key = hcm.get_demo_waypoints()
		if type(wps_key)~='string' then
			print('BAD WAYPOINTS KEY TYPE')
		else
			print('Waypoints Key:', wps_key)
			local wps_demo = Config.demo.waypoints[wps_key]
			if type(wps_demo)~='table' then
				print('BAD WAYPOINTS KEY')
			else
				waypoints = wps_demo
			end
		end
	end
	io.write('Waypoints:\n', table.concat(waypoints, '\n'), '\n')
	-- Set the waypoints
	coroutine.resume(wp_thread, waypoints)

	-- Start walking
	motion_ch:send'hybridwalk'

end

function state.update()
  --print(state._NAME..' Update' )
  local t  = Body.get_time()
  local dt = t - t_update
  -- Save this at the last update time
  t_update = t

	-- Get any human given bias in the walk
	local pBias = hcm.get_teleop_walkbias()
	local pose = vector.pose(wcm.get_robot_pose())
	local status, velocity = coroutine.resume(wp_thread, pose, pBias)

	if not status then return'done' end

	mcm.set_walk_vel(velocity)

	--[[
	print('pose', pose)
	print('target_pose', target_pose)
	print('dist', dist)
	print('vel', vel)
	--]]

end

function state.exit()
  print(state._NAME..' Exit' )
	mcm.set_walk_vel({0,0,0})
end

return state
