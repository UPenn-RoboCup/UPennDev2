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
local pathIter, qGoal

local timeout = 10.0
local t_entry, t_update, t_exit
local tr0

--
local t_cmd, CMD_INTERVAL = 0, 0.1

function state.entry()
  print(state._NAME..' Entry' )
  -- Update the time of entry
  local t_entry_prev = t_entry -- When entry was previously called
  t_entry = Body.get_time()


	-- -- Override for now -- --
	local pose = wcm.get_robot_pose()
	local poseRel = util.pose_global({.2,0,0}, pose)
	wcm.set_drill_pos({poseRel.x,poseRel.y,0})
	-- --   Override end   -- --

	-- Find initial guess of where we want to go
	local pos = wcm.get_drill_pos()
	local rot = wcm.get_drill_rot()
	tr0 = T.from_flat(pos,rot)
	--
	local pose = vector.pose(wcm.get_robot_pose())
	local trPose = T.trans(pose.x,pose.y,0) * T.rotZ(pose.a)
	local trRel = T.inv(trPose) * tr0
	local trRel6  = vector.new( T.position6D(trRel) )
	local poseRel = vector.pose{trRel6[1],trRel6[2],trRel6[6]}

	-- Goto the position first, always
	local qArm = Body.get_command_position()
	local goal_pos = vector.slice(trRel6,1,3)
	print('GOAL',goal_pos)
	pathIter, qGoal = planner:line_iter(qArm,goal_pos)

end

function state.update()
	-- Get the time of update
  local t  = Body.get_time()
  local dt = t - (t_update or t_entry)
  -- Save this at the last update time
  t_update = t

	--print(state._NAME..' Update', t, dt )
  --if (t-t_entry)>timeout then return'timeout' end

	-- Find where we want to go
	local pos = wcm.get_drill_pos()
	local rot = wcm.get_drill_rot()
	local tr = T.from_flat(pos,rot)
	-- TODO: Check the difference between the transforms

	-- Convert to local coordinates
	local pose = vector.pose(wcm.get_robot_pose())
	local trPose = T.trans(pose.x,pose.y,0) * T.rotZ(pose.a)
	local trRel = T.inv(trPose) * tr
	local trRel6  = vector.new( T.position6D(trRel) )
	local poseRel = vector.pose{trRel6[1],trRel6[2],trRel6[6]}
	--print('bodyGrab | RelObject',trRel6)
	--[[
	print('\nbodyGrab | My Pose',pose)
	print('bodyGrab | Drill Pose',pos)
	io.write('bodyGrab | Relative Transform\n',T.tostring(trRel),'\n')
	print('bodyGrab | Rel Pose2',poseRel)
	--]]

	-- Replan the grab as necessary
	--if obj_model_too_diff then return'grab' end
	-- TODO: May need to actually move to the object...

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
