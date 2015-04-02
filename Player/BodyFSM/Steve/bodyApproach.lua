local state = {}
state._NAME = ...
local Body   = require'Body'
local util   = require'util'
local vector = require'vector'

local t_entry, t_update, t_exit
local nwaypoints, wp_id
local waypoints = {}

local dist_threshold = 0.05
local angle_threshold = 5 * DEG_TO_RAD
local function robocup_approach(target_pose, pose)
  local maxStep = 0.08
  local maxTurn = 0.15

  -- Distance to the waypoint
  local rel_pose = util.pose_relative(target_pose, pose)
  local rel_dist = math.sqrt(math.pow(rel_pose.x,2)+math.pow(rel_pose.y,2))
	
	if rel_dist<dist_threshold and math.abs(rel_pose.a)<angle_threshold then
		return nil, {0,0,0}
	end

  -- calculate walk step velocity based on ball position
  local vStep = vector.new{
		util.procFunc(rel_pose[1]*0.5, 0, maxStep),
		util.procFunc(rel_pose[2]*0.5, 0, maxStep),
		0
	}
  -- Reduce speed based on how far away from the waypoint we are
  if rel_dist < 0.04 then maxStep = 0.02 end
  local scale = math.min(maxStep/math.sqrt(vStep[1]^2+vStep[2]^2), 1)
  vStep = scale * vStep

  return rel_dist, vStep
end

function state.entry()
  print(state._NAME..' Entry' )
  -- Update the time of entry
  local t_entry_prev = t_entry -- When entry was previously called

  t_entry = Body.get_time()
  t_update = t_entry

	-- Start walking
	motion_ch:send'hybridwalk'
	
end

function state.update()
  --print(state._NAME..' Update' )
  local t  = Body.get_time()
  local dt = t - t_update
  -- Save this at the last update time
  t_update = t

	local pose = wcm.get_robot_pose()
	local target_pose = vector.pose{1,0,0}
	local dist, vel = robocup_approach(target_pose, pose)
	
	mcm.set_walk_vel(vel)
	
	if not dist then return'done' end
	
	--[[
	print('pose', pose)
	print('target_pose', target_pose)
	print('dist', dist)
	print('vel', vel)
	--]]

end

function state.exit()
  print(state._NAME..' Exit' )
end

return state
