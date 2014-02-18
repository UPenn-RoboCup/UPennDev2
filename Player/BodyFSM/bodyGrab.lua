local state = {}
state._NAME = ...

local Body = require'Body'
local T = require'libTransform'
local K = Body.Kinematics
local vector = require'vector'
local P = require'libPlan'
local planner = P.new_planner(K,Body.servo.min_rad,Body.servo.max_rad)
local pathIter, qGoal

--local timeout = 10.0
local t_entry, t_update, t_exit
local tr0

function state.entry()
  print(state._NAME..' Entry' )
	init_stages()
  -- Update the time of entry
  local t_entry_prev = t_entry -- When entry was previously called
  t_entry = Body.get_time()
	-- Find initial guess of where we want to go
	local pos = wcm.get_drill_pos()
	local rot = wcm.get_drill_rot()
	tr0 = T.from_flat(pos,rot)
	-- Goto the position first, always
	local qArm = Body.get_command_position()
	pathIter, qGoal = planner:line_iter(qArm,pos)
end

function state.update()
  --print(state._NAME..' Update' )
  -- Get the time of update
  local t  = Body.get_time()
  local dt = t - (t_update or t_entry)
  -- Save this at the last update time
  t_update = t
  if (t-t_entry)>timeout then return'timeout' end

	-- Find where we want to go
	local pos = wcm.get_drill_pos()
	local rot = wcm.get_drill_rot()
	local tr = T.from_flat(pos,rot)
	-- TODO: Check the difference between the transforms

	-- Convert to local coordinates
	local pose = vector.pose(wcm.get_robot_pose())
	local trPose = T.trans(pose.x,pose.y,0) * T.rotZ(pose.a)
	local trRel = libTransform.inv(trPose) * tr
	io.write('bodyGrab | Relative Transform\n',T.tostring(trRel))

	-- Replan the grab as necessary
	--if obj_model_too_diff then return'grab' end
	-- TODO: May need to actually move to the object...

	-- Move the arm
	local qArm = Body.get_command_position()
	local qArmCmd = pathIter(qArm)
	-- TODO: If done positioning, then change to orierntation
	if not qArmCmd then return'done' end
	--Body.set_command_position(qArmCmd)

end

function state.exit()
  print(state._NAME..' Exit' )
  t_exit = Body.get_time()
end

return state
