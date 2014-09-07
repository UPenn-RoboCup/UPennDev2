local state = {}
state._NAME = ...

local Body = require'Body'
local libMap = require'libMap'
local util = require'util'
local vector = require'vector'
require'wcm'

-- TODO: Should be in the Config
local USE_GPS = true
local DO_EXPORT = false
local USE_ODOM = false

-- Replan every timeout
local timeout = 5.0
local t_entry, t_update, t_exit
local map, path, finished_path, cur_wp, wp_id, path_sz, cur_goal

-- The map is static, so import it once
map = libMap.open_map(HOME..'/Data/map.ppm')
-- Perform the convolution so that
-- the girth of the robot will not collide with walls
map:grow()

-- Export for MATLAB
if DO_EXPORT==true then
	libMap.export(map.cost,'cost.raw')
	libMap.export(map.cost_to_go,'cost_to_go.raw')
end

local function robocup_follow(pose, target_pose)
  local maxStep = 0.5
  local maxTurn = 0.2
  local dist_threshold = 0.025
  local angle_threshold = .1
	local stepScale = 4
	local aTurnSpeed = .25

  -- Distance to the waypoint
  local rel_pose = util.pose_relative(target_pose,pose)
  rel_pose[3] = util.mod_angle(rel_pose[3])
  local rel_dist =
		math.sqrt(rel_pose[1]*rel_pose[1] + rel_pose[2]*rel_pose[2])

  -- Angle towards the waypoint
  local aTurn = util.mod_angle(math.atan2(rel_pose[2],rel_pose[1]))

  -- calculate walk step velocity based on ball position
  local vStep =
		vector.new{stepScale * rel_pose[1], stepScale * rel_pose[2],0}

  -- Reduce speed based on how far away from the waypoint we are
  local scale = math.min(maxStep/math.sqrt(vStep[1]^2+vStep[2]^2), 1)
  vStep = scale * vStep

  -- If we are close to the waypoint and have the correct angle threshold
	-- then we are fine
  -- TODO: Only with the last point do we care about the angle
  --print('Relative distances',rel_dist,rel_pose)
  if rel_dist<dist_threshold then
    if math.abs(rel_pose[3])<angle_threshold then
    	-- if not the last waypoint, then we are done with this waypoint
      return vector.zeros(3), true
		end
		-- Turn towards the waypoint
    vStep[3] = aTurnSpeed * rel_pose[3]
  end
  return vStep, false
end

function state.entry()
  print(state._NAME..' Entry' )
  -- Update the time of entry
  local t_entry_prev = t_entry
  t_entry = Body.get_time()
  t_update = t_entry

	-- Check the goal, and update if needed
	local obj_pose = wcm.get_ball_pose()
	local obj_offset = vector.pose{-0.3,-0.2,0}
	local target_pose = util.pose_global(obj_offset,obj_pose)
	wcm.set_map_goal(target_pose)
	if target_pose~=cur_goal then
		print("NEW GOAL",target_pose)
		cur_goal = target_pose
		map:new_goal(target_pose)
	end

	-- Plan the from our current position
	local pose = Body.get_pose()
	path = map:new_path(pose, DO_EXPORT and 'path.raw')
	path_sz = #path
	cur_wp = table.remove(path)
	wp_id = 1
	finished_path = #path==0
	wcm.set_map_waypoint(cur_wp)

end

function state.update()
  --print(state._NAME..' Update',cur_wp)
  -- Get the time of update
  local t  = Body.get_time()
  local dt = t - t_update
  -- Save this at the last update time
  t_update = t
  if t-t_entry > timeout then return'timeout' end

	-- Grab the pose
	local pose = Body.get_pose()

	-- Find our velocity
	local velocity, near_wp = robocup_follow(pose,cur_wp)
	-- Grab the next waypoint
	if near_wp then
		finished_path = #path==0
		-- Done when finished the path
		if finished_path then
			mcm.set_walk_vel{0,0,0}
			return'done'
		end
		cur_wp = table.remove(path)
		wp_id = wp_id + 1
		wcm.set_map_waypoint(cur_wp)
	end

	-- Set the velocity
	mcm.set_walk_vel(velocity)

end

function state.exit()
  print(state._NAME..' Exit' )
  t_exit = Body.get_time()
end

return state
