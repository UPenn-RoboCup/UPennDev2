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

local mc_inputs = manipulation_controller.THOR_MC_U
local mc_outputs = manipulation_controller.THOR_MC_Y
	
local t = Platform.get_time()

function Manipulation_state.new(...)
	local o = Motion_state.new(...)
	return setmetatable(o, Manipulation_state)
end

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

local function set_real_T_array_from_table(real_T_array, array_offset, number_array)
	for count=1, #number_array 
		do manipulation_controller.real_T_setitem(real_T_array,array_offset-1+count,number_array[count])
	end
end

local function real_T_array_to_table(real_T_array, array_first, array_last)
	v = {}
	for index=array_first, array_last
		do 
			local val = manipulation_controller.real_T_getitem(real_T_array,index)
			n = index-array_first+1
			v[n] = val
			print(index..' = '..val)
				
	end
	return v
end

function Manipulation_state:entry()
	
end

function Manipulation_state:update()
	
	local dt = Platform.get_time() - t
  	t = Platform.get_time()
	
	manipulation_controller.THOR_MC_U.JointSpaceGoalsEnable = true
	manipulation_controller.THOR_MC_U.JointSpaceGoalType = false
	
	--set_real_T_array_from_table(mc_inputs.JointSpaceVelocityGoal, 1, mcm:get_desired_joint_velocity(joint.waist))
	--set_real_T_array_from_table(mc_inputs.JointSpaceVelocityGoal, 2, mcm:get_desired_joint_velocity(joint.r_arm))
	--set_real_T_array_from_table(mc_inputs.JointSpaceVelocityGoal, 8, mcm:get_desired_joint_velocity(joint.l_arm))
	
	--set_real_T_array_from_table(mc_inputs.JointSpacePositionGoal, 1, mcm:get_desired_joint_position(joint.waist))
	--set_real_T_array_from_table(mc_inputs.JointSpacePositionGoal, 2, mcm:get_desired_joint_position(joint.r_arm))
	--set_real_T_array_from_table(mc_inputs.JointSpacePositionGoal, 8, mcm:get_desired_joint_position(joint.l_arm))
	
	
	set_real_T_array_from_table(manipulation_controller.THOR_MC_U.JointSpaceVelocityGoal,0,{0.1,0.2,0.3,0.4,0.5,0.6})
         --set_real_T_array_from_table(mc_inputs.JointSpacePositionGoal, 0, {0.0,0.2,0.3,0.4,0.5,0.6,0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8})
	
	--set_real_T_array_from_table(mc_inputs.JointSpacePositionGoal, 8, mcm:get_desired_joint_position(joint.l_arm))
	
	--set_real_T_array_from_table(mc_inputs.TaskSpaceVelocityGoal, 0, mcm:get_desired_r_hand_twist())
	--set_real_T_array_from_table(mc_inputs.TaskSpaceVelocityGoal, 6, mcm:get_desired_l_hand_twist())
	
	--set_real_T_array_from_table(mc_inputs.TaskSpacePositionsGoal, 0, transform.get_array(transform.pose(mcm:get_desired_r_hand_pose())))
	--set_real_T_array_from_table(mc_inputs.TaskSpacePositionsGoal, 16, transform.get_array(transform.pose(mcm:get_desired_l_hand_pose())))
	
	--set_real_T_array_from_table(mc_inputs.JointPositions, 1, pcm:get_joint_position(joint.waist))
	--set_real_T_array_from_table(mc_inputs.JointPositions, 2, pcm:get_joint_position(joint.r_arm))
	--set_real_T_array_from_table(mc_inputs.JointPositions, 8, pcm:get_joint_position(joint.l_arm))
	
	--set_real_T_array_from_table(mc_inputs.EndForceTorques, 0, pcm:get_r_hand_wrench())
	--set_real_T_array_from_table(mc_inputs.EndForceTorques, 6, pcm:get_l_hand_wrench())				     
								     
	
	manipulation_controller.THOR_MC_U.TimeStep = dt
	print(manipulation_controller.THOR_MC_U.TimeStep)
	manipulation_controller.THOR_MC_step()

	
	if  manipulation_controller.THOR_MC_Y.Faulted == true then
	--handle faults and pass manipulation_controller.THOR_MC_Y.ErrorCode
	print("faulted")
	end
	
	--this long print statement allows the print to work even when curses is running, don't ask me why
	print("                                                                                                                                                       set joint vels")
	real_T_array_to_table(manipulation_controller.THOR_MC_Y.JointVelocityCmds,0,13)
	--self.dcm:set_joint_velocity(real_T_array_to_table(mc_outputs.JointVelocityCmds,1,1), joint.waist)
	--self.dcm:set_joint_velocity(real_T_array_to_table(mc_outputs.JointVelocityCmds,2,7), joint.r_arm)
	--self.dcm:set_joint_velocity(real_T_array_to_table(mc_outputs.JointVelocityCmds,7,13), joint.l_arm)
	--print("set joint pos")
	--self.dcm:set_joint_position(real_T_array_to_table(mc_outputs.JointPositionCmds,1,1), joint.waist)
	--self.dcm:set_joint_position(real_T_array_to_table(mc_outputs.JointPositionCmds,2,7), joint.l_arm)
	--self.dcm:set_joint_position(real_T_array_to_table(mc_outputs.JointPositionCmds,8,13), joint.r_arm)

	--print(self.dcm:get_joint_velocity(joint.l_arm))
	
	--self.dcm:set_joint_position({0.1,0.2,0.3,0.4,0.5,0.6}, joint.r_arm)
	
	local step_delta = Platform.get_time() - t
 	--print("manipulation state step "..step_delta)	

end

function Manipulation_state:exit()
	--manipulation_controller.THOR_MC_terminate()
end

return Manipulation_state