require('Manipulation_state')
require('Config')

--------------------------------------------------------------------------
-- Locomotion Slave Controller
--------------------------------------------------------------------------

locomotion_slave = Manipulation_state.new('locomotion_slave')

local joint = Config.joint
locomotion_slave:set_joint_access(0, joint.all)
locomotion_slave:set_joint_access(1, joint.upperbody)
locomotion_slave:set_joint_access(0, joint.head)

-- default parameters
locomotion_slave.parameters = {
}

-- config parameters
locomotion_slave:load_parameters()

function locomotion_slave:entry()
	Manipulation_state.entry(self)

	-- set proper joint gains for position control
	self.dcm:set_joint_p_gain(0.8, joint.upperbody)
	self.dcm:set_joint_i_gain(0.1, joint.upperbody)
	self.dcm:set_joint_d_gain(0.005, joint.upperbody)

	-- enable end-effector position control
	self:set_joint_goal_enabled(false)
	self:set_task_goal_enabled(true)
	self:set_task_goal_type(true, true)

	-- initialize end-effector goals
	mcm:set_desired_l_hand_pose(pcm:get_l_hand_pose())
	mcm:set_desired_r_hand_pose(pcm:get_r_hand_pose())
end

function locomotion_slave:update()
	Manipulation_state.update(self)
end

function locomotion_slave:exit()
	Manipulation_state.exit(self)
end

return locomotion_slave
