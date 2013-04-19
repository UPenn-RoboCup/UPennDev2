require('Motion_state')
require('Config')

--------------------------------------------------------------------------
-- Carry Controller
--------------------------------------------------------------------------

carry = Motion_state.new('carry')

local dcm = carry.dcm
local joint = Config.joint
carry:set_joint_access(0, joint.all)
carry:set_joint_access(1, joint.upperbody)
carry:set_joint_access(0, joint.head)

-- default parameters
carry.parameters = {
}

-- config parameters
carry:load_parameters()

function carry:entry()
end

function carry:update()
end

function carry:exit()
end

return carry
