require('Motion_state')
require('Config')
require('Transform')
require('manipulation_controller')
--------------------------------------------------------------------------
-- Manipulation Controller
--------------------------------------------------------------------------

Manipulation_state = Motion_state.new('Manipulation_state')
local joint = Config.joint

Manipulation_state:set_joint_access(0, 'all')
Manipulation_state:set_joint_access(1, 'upperbody')
Manipulation_state:set_joint_access(0, 'head')

local dcm = Manipulation_state.dcm
local mcm = Manipulation_state.mcm
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

function set_real_T_array_from_table(real_T_array, number_array)
	for count=1, #number_array 
		do manipulation_controller.real_T_setitem(real_T_array,count,number_array[count])
	end
end

function set_table_from_real_T_array(number_array,real_T_array)
	for count=1, #number_array 
		do number_array[count] = manipulation_controller.real_T_getitem(real_T_array,count)
	end
end

function Manipulation_state:entry()
	manipulation_controller.THOR_MC_initialize()
	--mc_inputs = new ExternalInputs_THOR_MC()
end

function Manipulation_state:update()
	
	local dt = Platform.get_time() - t
  	t = Platform.get_time()
	
	set_real_T_array_from_table(mc_inputs.JointSpaceVelocityGoal, {mcm:get_desired_joint_velocity(joint.waist)})
						        --[[table.concat(0 ,{0,0,0,0,0,0,0,0},mcm:get_desired_joint_velocity(joint.waist)))
									           ,mcm:desired_joint_velocity(joint.r_arm)
									           ,mcm:desired_joint_velocity(joint.l_arm) ) )
												
	
	set_real_T_array_from_table(mc_inputs.JointSpacePositionGoal,
						   { 0, mcm:desired_joint_velocity({ joint.waist, 
												joint.r_arm,
												joint.l_arm } ) } )

	set_real_T_array_from_table(mc_inputs.TaskSpaceVelocityGoal,
						  { mcm:desired_r_hand_twist(), mcm:desired_l_hand_twist() } )
						  
	set_real_T_array_from_table(mc_inputs.TaskSpacePositionsGoal,
						  { Transform:pose(mcm:desired_r_hand_pose()),
								     Transform:pose(mcm:desired_l_hand_pose())  } )
								     
        set_real_T_array_from_table(mc_inputs.JointPositions,
						 { 0, pcm:joint_position({ joint.waist, 
											joint.r_arm,
											joint.l_arm } ) } )
								     
	set_real_T_array_from_table(mc_inputs.EndForceTorques,
						{ pcm:r_hand_wrench(), pcm:l_hand_wrench() } )							     
								     
	
	
	add manipulation step with time
	
	]]--
	manipulation_controller.TimeStep = dt
	
	
	
	manipulation_controller.THOR_MC_step()

	
	if  manipulation_controller.THOR_MC_Y.Faulted == true then
	--handle faults and pass manipulation_controller.THOR_MC_Y.ErrorCode
	end
	
	--does VT need arm transforms? manipulation_controller.THOR_MC_Y.ArmTransforms

	--can we do both postion and vel or not?
	-dcm.set_joint_velocity( manipulation_controller.THOR_MC_Y.JointVelocityCmds[1],  { joint.waist, 
											    joint.r_arm,
											    joint.l_arm } )

	dcm.set_joint_position( manipulation_controller.THOR_MC_Y.JointPositionCmds[1],  { joint.waist, 
											    joint.r_arm,
											    joint.l_arm } )]]
										    
	local step_delta = Platform.get_time() - t
	--print("manipulation state step "..step_delta)	

end



function Manipulation_state:exit()
end

return Manipulation_state
