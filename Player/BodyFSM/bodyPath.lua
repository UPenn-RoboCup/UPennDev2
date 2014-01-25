local state = {}
state._NAME = ...

local Body = require'Body'
local libMap = require'libMap'
local util = require'util'
local vector = require'vector'
require'wcm'

local timeout = 10.0  -- Replan every 10 seconds
local t_entry, t_update, t_exit

-- The map is static, so import it once
local map = libMap.open_map'map.ppm'
-- Perform the convolution so that
-- the girth of the robot will not collide with walls
map:grow()

-- Make the initial cost to go
local goal = wcm.get_map_goal()
map:new_goal(goal)
-- Access the path persistently
local path, finished_path, cur_wp, wp_id, path_sz

-- Render so that I can see it :)
local c_map = map:render'png'
local f_map = io.open('cur_map.png','w')
f_map:write(c_map)
f_map:close()

-- Export for MATLAB
libMap.export(map.cost,'cost.raw')
libMap.export(map.cost_to_go,'cost_to_go.raw')

local function robocup_follow( pose, target_pose )
  local maxStep = 0.15
  local maxTurn = 0.15
  local dist_threshold = 0.025
  local angle_threshold = .1
	local stepScale = .25
	local aTurnSpeed = .25

  -- Distance to the waypoint
  local rel_pose = util.pose_relative(target_pose,pose)
  rel_pose[3] = util.mod_angle(rel_pose[3])
  local rel_dist = math.sqrt(rel_pose[1]*rel_pose[1] + rel_pose[2]*rel_pose[2])

  -- Angle towards the waypoint
  local aTurn = util.mod_angle(math.atan2(rel_pose[2],rel_pose[1]))

  -- calculate walk step velocity based on ball position
  local vStep = vector.new{stepScale * rel_pose[1], stepScale * rel_pose[2],0}

  -- Reduce speed based on how far away from the waypoint we are
  local scale = math.min(maxStep/math.sqrt(vStep[1]^2+vStep[2]^2), 1)
  vStep = scale * vStep

  -- If we are close to the waypoint and have the right angle threshold, we are fine
  -- TODO: Only with the last point do we care about the angle
  --print('Relative distances',rel_dist,rel_wp.a*180/math.pi)
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
  local t_entry_prev = t_entry -- When entry was previously called
  t_entry = Body.get_time()
  t_update = t_entry
	
	-- Check the goal, and update if needed
	local cur_goal = wcm.get_map_goal()
	if goal~=cur_goal then
		goal = cur_goal
		map:new_goal(goal)
	end
	
	-- Plan the from our current position
	local pose = wcm.get_robot_pose()
	-- Store the path information persistently
	path = map:new_path(pose)
	path_sz = #path
	cur_wp = table.remove(path)
	wp_id = 1
	finished_path = #path==0
	--[[
	print('Start',pose)
	print('Goal',goal)
	for i,p in ipairs(path) do
		print('Waypoint',i,p)
	end
	--]]
	
end

function state.update()
  --  print(state._NAME..' Update' ) 
  -- Get the time of update
  local t  = Body.get_time()
  local dt = t - t_update
  -- Save this at the last update time
  t_update = t
  if t-t_entry > timeout then return'timeout' end

	-- Find our velocity
	local pose = wcm.get_robot_pose()
	local velocity, near_wp = robocup_follow(pose,cur_wp)
	-- Grab the next waypoint
	if near_wp then
		cur_wp = table.remove(path)
		wp_id = wp_id + 1
		finished_path = #path==0
	end
	--print('Path following',wp_id,path_sz,velocity)

	-- Done when finished the path
  if finished_path then
		mcm.set_walk_vel{0,0,0}
		return'done'
	end

	-- Set the velocity
	mcm.set_walk_vel(velocity)

end

function state.exit()
  print(state._NAME..' Exit' )
  t_exit = Body.get_time()
end

return state
