require('Motion_state')
require('Platform')
require('Config')
require('manipulation_controller')
require('mcm')
require('pcm')
local transform = require('Transform')

--------------------------------------------------------------------------
-- Manipulation Controller Base Class
--------------------------------------------------------------------------

Manipulation_state = {}
Manipulation_state.__index = Manipulation_state
Manipulation_state.__mtstring = 'Manipulation_state'
setmetatable(Manipulation_state, Motion_state)

local joint = Config.joint
local t = Platform.get_time()

local set_real_T_array_from_table, real_T_array_to_table
local set_boolean_T_array_from_table, boolean_T_array_to_table

function Manipulation_state.new(...)
	local o = Motion_state.new(...)
	return setmetatable(o, Manipulation_state)
end

function Manipulation_state:set_joint_goal_enabled(enabled)
	manipulation_controller_inputs.JointSpaceGoalEnabled = enabled
end

function Manipulation_state:set_joint_goal_type(is_position_else_velocity)
	manipulation_controller_inputs.JointSpaceGoalType = is_position_else_velocity
end

function Manipulation_state:set_task_goal_enabled(enabled)
	manipulation_controller_inputs.TaskSpaceGoalEnabled = enabled
end

function Manipulation_state:set_task_goal_type(is_right_position_else_velocity,  is_left_position_else_velocity)
	set_boolean_T_array_from_table(manipulation_controller_inputs.TaskSpaceGoalType,
							0,
							{ is_right_position_else_velocity , is_left_position_else_velocity })
end

function Manipulation_state:set_linear_tolerance(linear_tolerance)
	manipulation_controller_inputs.PositionLoopLinearTolerance = linear_tolerance
end

function Manipulation_state:set_angular_tolerance(angular_tolerance)
	manipulation_controller_inputs.PositionLoopAngularTolerance = angular_tolerance
end

function Manipulation_state:set_gripper_positition(gripper_position)
	--manipulation_controller_inputs.
end

function Manipulation_state:set_relative_motion_mode(relative_motion_mode)
	manipulation_controller_inputs.RelativeMotionMode = relative_motion_mode
end

--array of 14 booleans, see API
function Manipulation_state:set_joint_space_motion_locks(joint_space_motion_locks)
	set_boolean_T_array_from_table(manipulation_controller_inputs.JointSpaceMotionLocks,0,joint_space_motion_locks)
end

-- array of 24 booleans, see API
function Manipulation_state:set_task_space_motion_locks(task_space_motion_locks)
	set_boolean_T_array_from_table(manipulation_controller_inputs.TaskSpaceMotionLocks,0,task_space_motion_locks)
end

--array of 54 booleans, see API
function Manipulation_state:set_inequality_constraints(inequality_constraints)
	set_boolean_T_array_from_table(manipulation_controller_inputs.InequalityConstraintsEnabled,0,inequaility_constraints)
end

--array of 12 floats, see API
function Manipulation_state:set_obstacle_shell_position(obstacle_shell_position)
	set_real_T_array_from_table(manipulation_controller_inputs.ObstacleShellPosition,0,obstacle_shell_position)
end

function Manipulation_state:entry()
	t = Platform.get_time()
end

function Manipulation_state:update()
	
	local dt = Platform.get_time() - t
  	t = Platform.get_time()
	
	set_real_T_array_from_table(manipulation_controller_inputs.JointSpaceVelocityGoal, 1, mcm:get_desired_joint_velocity(joint.waist))
	set_real_T_array_from_table(manipulation_controller_inputs.JointSpaceVelocityGoal, 2, mcm:get_desired_joint_velocity(joint.r_arm))
	set_real_T_array_from_table(manipulation_controller_inputs.JointSpaceVelocityGoal, 8, mcm:get_desired_joint_velocity(joint.l_arm))
	
	set_real_T_array_from_table(manipulation_controller_inputs.JointSpacePositionGoal, 1, mcm:get_desired_joint_position(joint.waist))
	set_real_T_array_from_table(manipulation_controller_inputs.JointSpacePositionGoal, 2, mcm:get_desired_joint_position(joint.r_arm))
	set_real_T_array_from_table(manipulation_controller_inputs.JointSpacePositionGoal, 8, mcm:get_desired_joint_position(joint.l_arm))

	set_real_T_array_from_table(manipulation_controller_inputs.JointSpacePositionGoal, 8, mcm:get_desired_joint_position(joint.l_arm))
	
	set_real_T_array_from_table(manipulation_controller_inputs.TaskSpaceVelocityGoal, 0, mcm:get_desired_r_hand_twist())
	set_real_T_array_from_table(manipulation_controller_inputs.TaskSpaceVelocityGoal, 6, mcm:get_desired_l_hand_twist())
	
	set_real_T_array_from_table(manipulation_controller_inputs.TaskSpacePositionsGoal, 0, 
					          transform.get_array(transform.pose(mcm:get_desired_r_hand_pose())))
	set_real_T_array_from_table(manipulation_controller_inputs.TaskSpacePositionsGoal, 16, 
	                                          transform.get_array(transform.pose(mcm:get_desired_l_hand_pose())))
	
	set_real_T_array_from_table(manipulation_controller_inputs.JointPositions, 1, pcm:get_joint_position(joint.waist))
	set_real_T_array_from_table(manipulation_controller_inputs.JointPositions, 2, pcm:get_joint_position(joint.r_arm))
	set_real_T_array_from_table(manipulation_controller_inputs.JointPositions, 8, pcm:get_joint_position(joint.l_arm))
	
	set_real_T_array_from_table(manipulation_controller_inputs.EndForceTorques, 0, pcm:get_r_hand_wrench())
	set_real_T_array_from_table(manipulation_controller_inputs.EndForceTorques, 6, pcm:get_l_hand_wrench())			     
								     
	
	manipulation_controller.THOR_MC_U.TimeStep = dt
	manipulation_controller.THOR_MC_step()

	
	if  manipulation_controller.THOR_MC_Y.Faulted == true then
	--handle faults and pass manipulation_controller.THOR_MC_Y.ErrorCode
	print("faulted")
	end
	
	self.dcm:set_joint_velocity(real_T_array_to_table(manipulation_controller_outputs.JointVelocityCmds,1,1), joint.waist)
	self.dcm:set_joint_velocity(real_T_array_to_table(manipulation_controller_outputs.JointVelocityCmds,2,7), joint.l_arm)
	self.dcm:set_joint_velocity(real_T_array_to_table(manipulation_controller_outputs.JointVelocityCmds,8,13), joint.r_arm)
	
	self.dcm:set_joint_position(real_T_array_to_table(manipulation_controller_outputs.JointPositionCmds,1,1), joint.waist)
	self.dcm:set_joint_position(real_T_array_to_table(manipulation_controller_outputs.JointPositionCmds,2,7), joint.l_arm)
	self.dcm:set_joint_position(real_T_array_to_table(manipulation_controller_outputs.JointPositionCmds,8,13), joint.r_arm)
	
	local step_delta = Platform.get_time() - t

end

function Manipulation_state:exit()
	
end

function set_real_T_array_from_table(real_T_array, array_offset, number_array)
	for count=1, #number_array 
		do manipulation_controller.real_T_setitem(real_T_array,array_offset-1+count,number_array[count])
	end
end

function real_T_array_to_table(real_T_array, array_first, array_last)
	local v = {}
	for index=array_first, array_last
		do 
			local val = manipulation_controller.real_T_getitem(real_T_array,index)
			n = index-array_first+1
			v[n] = val
	end
	return v
end

function set_boolean_T_array_from_table(boolean_T_array, array_offset, boolean_array)
	for count=1, #boolean_array 
		do manipulation_controller.boolean_T_setitem(boolean_T_array,array_offset-1+count,boolean_array[count])
	end
end

function boolean_T_array_to_table(boolean_T_array, array_first, array_last)
	local v = {}
	for index=array_first, array_last
		do 
			local val = manipulation_controller.real_T_getitem(boolean_T_array,index)
			n = index-array_first+1
			v[n] = val
	end
	return v
end

return Manipulation_state
