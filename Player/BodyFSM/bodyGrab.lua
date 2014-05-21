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
local pathIter, qGoal, relative_pick_tr0

local timeout = 10.0
local t_entry, t_update, t_exit
--
local t_cmd, CMD_INTERVAL = 0, 0.1

local DEBUG = true

local function get_pick_transform()
	local pose = vector.pose(wcm.get_robot_pose())
	local pose_arm = util.pose_global({0.14,0,0},pose)
	local obj_pose = vector.pose(wcm.get_ball_pose())
	local pose_rel = util.pose_relative(obj_pose,pose_arm)

	-- FOR DEBUG ONLY
	if DEBUG then pose_rel = vector.pose{.25,.1,0} end

	local relative_pick_tr =
		T.trans(pose_rel.x,pose_rel.y,-0.2) * T.rotY(180*DEG_TO_RAD)
	return relative_pick_tr
end

function state.entry()
  print(state._NAME..' Entry' )
  -- Update the time of entry
  local t_entry_prev = t_entry -- When entry was previously called
  t_entry = Body.get_time()
	--
	relative_pick_tr0 = get_pick_transform()
	local qArm = Body.get_command_position()
	pathIter, qGoal = planner:line_iter(qArm,relative_pick_tr0,nil,nil,true)
	--pathIter, qGoal = planner:line_iter(qArm,relative_pick_tr0)
	--pathIter, qGoal = planner:line_stack(qArm,relative_pick_tr0)
	-- The joint follower works, and qGoal is correct
	--pathIter = planner:joint_iter(qArm,qGoal)
	--pathIter = planner:joint_stack(qArm,qGoal)
	--io.write('GOAL tr\n',T.tostring(relative_pick_tr0),'\n')
	--print('GOAL q',qGoal)
end

function state.update()
	-- Get the time of update
  local t  = Body.get_time()
  local dt = t - (t_update or t_entry)
  -- Save this at the last update time
  t_update = t

	--print(state._NAME..' Update', t, dt )
  --if (t-t_entry)>timeout then return'timeout' end
	-- TODO: Check if too far from the initial place
	--local relative_pick_tr = get_pick_position()

	-- Move the arm
	--local qArm = Body.get_command_position()
	local qArm = Body.get_position()
	local qArmCmd = pathIter(qArm)
	-- TODO: If done positioning, then change to orierntation
	if not qArmCmd then
		-- Set to the goal, since iterator may not hit, since within tolerance
		Body.set_command_position(qGoal)
		return'done'
	elseif type(qArmCmd)~='table' then
		pathIter = planner:joint_stack(qArm,qGoal,.25*DEG_TO_RAD)
		return
	end
	vector.new( qArmCmd )
	Body.set_command_position(qArmCmd)

end

function state.exit()
  print(state._NAME..' Exit' )
  t_exit = Body.get_time()
end

return state
