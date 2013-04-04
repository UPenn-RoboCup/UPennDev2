require('Motion_state')
require('Manipulation_state')
require('Config')
require('create_class')

--------------------------------------------------------------------------
-- Locomotion Slave Controller
--------------------------------------------------------------------------

locomotion_slave = createClass(Motion_state, Manipulation_state)
locomotion_slave._NAME = 'locomotion_slave'

local dcm = locomotion_slave.dcm
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
end

function locomotion_slave:update()

end

function locomotion_slave:exit()
end

return locomotion_slave
