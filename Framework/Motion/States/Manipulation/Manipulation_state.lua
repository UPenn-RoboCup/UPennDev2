require('Motion_state')
require('Config')
require('manipulation_controller')
require('mcm')
require('dcm')
require('pcm')
local transform = require('Transform')

--------------------------------------------------------------------------
-- Manipulation Controller
--------------------------------------------------------------------------

Manipulation_state = Motion_state.new('Manipulation_state')
local joint = Config.joint

Manipulation_state:set_joint_access(0, 'all')
Manipulation_state:set_joint_access(1, 'upperbody')
Manipulation_state:set_joint_access(0, 'head')

local dcm = Manipulation_state.dcm
local t = Platform.get_time()

local mc_inputs = manipulation_controller.THOR_MC_U
local mc_outputs = manipulation_controller.THOR_MC_Y

function Manipulation_state:set_joint_goal_enabled(enabled)
	mc_inputs.JointSpaceGoalEnabled = enabled
end

function Manipulation_state:set_joint_goal_type(is_position_else_velocity)
	mc_inputs.JointSpaceGoalType = is_position_else_velocity
end

function Manipulation_state:set_task_goal_enabled(enabled)
	mc_inputs.TaskSpaceGoalEnabled = enabled
end

function Manipulation_state:set_task_goal_type(is_right_position_else_velocity,  is_left_position_else_velocity)
	mc_inputs.TaskSpaceGoalType = { is_right_position_else_velocity , is_left_position_else_velocity }
end

--add modifiers for lineartolerance, angulartolerance, gripperpositioncommand, relativeMotionMode,
--jointspacemotionlocks, taskspacemotionlocks, inequalityconstraintsenabled,  obstableshellposition

function set_real_T_array_from_table(real_T_array, array_offset, number_array)
	for count=1, #number_array 
		do manipulation_controller.real_T_setitem(real_T_array,array_offset-1+count,number_array[count])
	end
end

function real_T_array_to_table(real_T_array, array_first, array_last)
	t = {}
	for index=array_first, array_last
		do 
			val = manipulation_controller.real_T_getitem(real_T_array,index)
			--print(index..' '..swig_type(val)..val)
			t[index] = val
		
	end
	return t
end

function Manipulation_state:entry()
	--mc_inputs = new ExternalInputs_THOR_MC()
end

function Manipulation_state:update()
	
	local dt = Platform.get_time() - t
  	t = Platform.get_time()
	
	set_real_T_array_from_table(mc_inputs.JointSpaceVelocityGoal, 1, mcm:get_desired_joint_velocity(joint.waist))
	set_real_T_array_from_table(mc_inputs.JointSpaceVelocityGoal, 2, mcm:get_desired_joint_velocity(joint.r_arm))
	set_real_T_array_from_table(mc_inputs.JointSpaceVelocityGoal, 8, mcm:get_desired_joint_velocity(joint.l_arm))
	
	--[[set_real_T_array_from_table(mc_inputs.JointSpacePositionGoal, 1, mcm:get_desired_joint_position(joint.waist))
	set_real_T_array_from_table(mc_inputs.JointSpacePositionGoal, 2, mcm:get_desired_joint_position(joint.r_arm))
	set_real_T_array_from_table(mc_inputs.JointSpacePositionGoal, 8, mcm:get_desired_joint_position(joint.l_arm))]]
	
	set_real_T_array_from_table(mc_inputs.TaskSpaceVelocityGoal, 0, mcm:get_desired_r_hand_twist())
	set_real_T_array_from_table(mc_inputs.TaskSpaceVelocityGoal, 6, mcm:get_desired_l_hand_twist())
	
	set_real_T_array_from_table(mc_inputs.TaskSpacePositionsGoal, 0, transform.get_array(transform.pose(mcm:get_desired_r_hand_pose())))
	set_real_T_array_from_table(mc_inputs.TaskSpacePositionsGoal, 16, transform.get_array(transform.pose(mcm:get_desired_l_hand_pose())))
	
	set_real_T_array_from_table(mc_inputs.JointPositions, 1, pcm:get_joint_position(joint.waist))
	set_real_T_array_from_table(mc_inputs.JointPositions, 2, pcm:get_joint_position(joint.r_arm))
	set_real_T_array_from_table(mc_inputs.JointPositions, 8, pcm:get_joint_position(joint.l_arm))
	
	set_real_T_array_from_table(mc_inputs.EndForceTorques, 0, pcm:get_r_hand_wrench())
	set_real_T_array_from_table(mc_inputs.EndForceTorques, 6, pcm:get_l_hand_wrench())				     
								     
	
	manipulation_controller.TimeStep = dt
	manipulation_controller.THOR_MC_step()

	
	if  manipulation_controller.THOR_MC_Y.Faulted == true then
	--handle faults and pass manipulation_controller.THOR_MC_Y.ErrorCode
	end
	
	--dcm:set_joint_velocity(real_T_array_to_table(mc_outputs.JointVelocityCmds,1,1), joint.waist)
	--dcm:set_joint_velocity(real_T_array_to_table(mc_outputs.JointVelocityCmds,2,7), joint.r_arm)
	--dcm:set_joint_velocity(real_T_array_to_table(mc_outputs.JointVelocityCmds,2,7), joint.j_arm)
										    
	local step_delta = Platform.get_time() - t
	--print("manipulation state step "..step_delta)	

end

function Manipulation_state:exit()
end

return Manipulation_state
