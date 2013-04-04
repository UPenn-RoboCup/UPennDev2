require('Motion_state')
require('Config')
require('Transform')
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

function Manipulation_state.new(name)
  local o = {_NAME = name}
  return setmetatable(o, Manipulation_state)
end

-- default parameters
Manipulation_state.parameters = {
}

-- config parameters
Manipulation_state:load_parameters()

function Manipulation_state:set_joint_goal_enabled(enabled)
	manipulation_controller.THOR_MC_U.JointSpaceGoalEnabled = enabled
end

function Manipulation_state:set_joint_goal_type(is_position_else_velocity)
	manipulation_controller.THOR_MC_U.JointSpaceGoalType = is_position_else_velocity
end

function Manipulation_state:set_task_goal_enabled(enabled)
	manipulation_controller.THOR_MC_U.TaskSpaceGoalEnabled = enabled
end

function Manipulation_state:set_task_goal_type(is_right_position_else_velocity,  is_left_position_else_velocity)
	manipulation_controller.THOR_MC_U.TaskSpaceGoalType = { is_right_position_else_velocity , is_left_position_else_velocity }
end

--add modifiers for lineartolerance, angulartolerance, gripperpositioncommand, relativeMotionMode,
--jointspacemotionlocks, taskspacemotionlocks, inequalityconstraintsenabled,  obstableshellposition

function Manipulation_state:entry()
end

function Manipulation_state:update()
	--
	
	local dt = Platform.get_time() - t
  	t = Platform.get_time()
	
	manipulation_controller.THOR_MC_U.JointSpaceVelocityGoal = { 0, mcm:desired_joint_velocity({ joint.waist, 
												joint.r_arm,
												joint.l_arm } ) }
	manipulation_controller.THOR_MC_U.JointSpacePositionGoal = { 0, mcm:desired_joint_position({ joint.waist, 
												joint.r_arm,
												joint.l_arm } ) }
	manipulation_controller.THOR_MC_U.TaskSpaceVelocityGoal = { mcm:desired_r_hand_twist(), mcm:desired_l_hand_twist() }

	manipulation_controller.THOR_MC_U.TaskSpacePositionsGoal = { Transform:pose(mcm:desired_r_hand_pose()),
								     Transform:pose(mcm:desired_l_hand_pose())  }

	manipulation_controller.THOR_MC_U.JointPositions = { 0, pcm:joint_position({ 	joint.waist, 
											joint.r_arm,
											joint.l_arm } ) }
	manipulation_controller.THOR_MC_U.EndForceTorques = { pcm:r_hand_wrench(), pcm:l_hand_wrench() }

	-- add manipulation step with time
	
	manipulation_controller.THOR_MC_U.TimeStep = dt
	
	manipulation_controller.THOR_MC_step()

	
	if  manipulation_controller.THOR_MC_Y.Faulted == true then
	--handle faults and pass manipulation_controller.THOR_MC_Y.ErrorCode
	end
	
	--does VT need arm transforms? manipulation_controller.THOR_MC_Y.ArmTransforms

	--can we do both postion and vel or not?
	dcm.set_joint_velocity( manipulation_controller.THOR_MC_Y.JointVelocityCmds[1],  { joint.waist, 
											    joint.r_arm,
											    joint.l_arm } )

	dcm.set_joint_position( manipulation_controller.THOR_MC_Y.JointPositionCmds[1],  { joint.waist, 
											    joint.r_arm,
											    joint.l_arm } )
											    
	local step_delta = Platform.get_time()-t
	print("manipulation state step "..step_delta)	

end



function Manipulation_state:exit()
end

return Manipulation_state
