local state = {}
state._NAME = ...

local Body = require'Body'
local T = require'libTransform'
local K = Body.Kinematics
local vector = require'vector'
local util = require'util'
local quaternion = require'quaternion'
local P = require'libPlan'
local planner = P.new_planner(K,Config.servo.min_rad,Config.servo.max_rad)
local pathIter, qGoal, relative_pick_pos0

local timeout = 10.0
local t_entry, t_update, t_exit
--
local t_cmd, CMD_INTERVAL = 0, 0.1

local DEBUG = true

local function get_pick_position()
	local pose = vector.pose(wcm.get_robot_pose())
	local pose_arm = util.pose_global({0.14,0,0},pose)
	local obj_pose = vector.pose(wcm.get_ball_pose())
	local pose_rel = util.pose_relative(obj_pose,pose_arm)
	local relative_pick_pos = vector.new{pose_rel.x,pose_rel.y,0.0}
	-- FOR DEBUG ONLY
	if DEBUG then relative_pick_pos = vector.new{.25,.1,0} end

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
	--print('pick_diff',pick_diff)
	if pick_diff>.1 then return'far' end

	-- Move the arm
	--local qArm = Body.get_command_position()
	local qArm = Body.get_position()
	local qArmCmd = pathIter(qArm)
	-- TODO: If done positioning, then change to orierntation
	if not qArmCmd then
		-- Set to the goal, since iterator may not hit, since within tolerance
		Body.set_command_position(qGoal)
		return'done'
	end
	vector.new( qArmCmd )
	--print(t,qArmCmd)
	Body.set_command_position(qArmCmd)

end

function state.exit()
  print(state._NAME..' Exit' )
  t_exit = Body.get_time()
end

return state
