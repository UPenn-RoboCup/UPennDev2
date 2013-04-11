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
end

function locomotion_slave:update()
	Manipulation_state.update(self)
end

function locomotion_slave:exit()
	Manipulation_state.exit(self)
end

return locomotion_slave