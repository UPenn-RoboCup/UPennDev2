local state = {}
state._NAME = ...

local Body = require'Body'
local T = require'libTransform'
local K = Body.Kinematics
local vector = require'vector'
local util = require'util'
local quaternion = require'quaternion'
local P = require'libPlan'
local planner = P.new_planner(K,Body.servo.min_rad,Body.servo.max_rad)
local pathIter, qGoal, relative_pick_pos0

local timeout = 10.0
local t_entry, t_update, t_exit
--
local t_cmd, CMD_INTERVAL = 0, 0.1

local function get_pick_position()
	local pose = vector.pose(wcm.get_robot_pose())
	local pose_arm = util.pose_global({0.14,0,0},pose)
	local obj_pose = vector.pose(wcm.get_drill_pose())
	local pose_rel = util.pose_relative(obj_pose,pose_arm)
	local relative_pick_pos = vector.new{pose_rel.x,pose_rel.y,-0.05}
	return relative_pick_pos
end

function state.entry()
  print(state._NAME..' Entry' )
  -- Update the time of entry
  local t_entry_prev = t_entry -- When entry was previously called
  t_entry = Body.get_time()

	relative_pick_pos0 = get_pick_position()
	print('GOAL',relative_pick_pos0)
	local qArm = Body.get_command_position()
	pathIter, qGoal = planner:line_iter(qArm,relative_pick_pos0)

	-- TODO: Full object orientation
	-- Find initial guess of where we want to go
	--[[
	local pos = wcm.get_drill_pos()
	local rot = wcm.get_drill_rot()
	tr0 = T.from_flat(pos,rot)
	local trPose = T.trans(pose.x,pose.y,0) * T.rotZ(pose.a)
	local trRel = T.inv(trPose) * tr0
	local trRel6  = vector.new( T.position6D(trRel) )
	--]]

end

function state.update()
	-- Get the time of update
  local t  = Body.get_time()
  local dt = t - (t_update or t_entry)
  -- Save this at the last update time
  t_update = t

	--print(state._NAME..' Update', t, dt )
  --if (t-t_entry)>timeout then return'timeout' end
	local relative_pick_pos = get_pick_position()
	-- Check if too far from the initial place
	local pick_diff = vector.norm(relative_pick_pos - relative_pick_pos0)
	--print('DIFF PICK',pick_diff)

	-- Timing
	-- TODO: Have some speed parameter, based on mm resolution
	if t-t_cmd>CMD_INTERVAL then
		t_cmd = t
		-- Move the arm
		local qArm = Body.get_command_position()
		local qArmCmd = pathIter(qArm)
		-- TODO: If done positioning, then change to orierntation
		if not qArmCmd then
			-- Set to the goal, since iterator may not hit, since within tolerance
			Body.set_command_position(qGoal)
			return'done'
		end
		vector.new( qArmCmd )
		Body.set_command_position(qArmCmd)
	end

end

function state.exit()
  print(state._NAME..' Exit' )
  t_exit = Body.get_time()
end

return state
