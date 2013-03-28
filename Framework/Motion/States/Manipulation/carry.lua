require('Motion_state')
require('Config')

--------------------------------------------------------------------------
-- Carry Controller
--------------------------------------------------------------------------

carry = Motion_state.new('carry')
carry:set_joint_access(0, 'all')
carry:set_joint_access(1, 'upperbody')
carry:set_joint_access(0, 'head')
local dcm = carry.dcm

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
