require('Motion_state')
require('Config')

--------------------------------------------------------------------------
-- Tele-Op Controller
--------------------------------------------------------------------------

teleop = Motion_state.new('teleop')
teleop:set_joint_access(0, 'all')
teleop:set_joint_access(1, 'upperbody')
teleop:set_joint_access(0, 'head')
local dcm = teleop.dcm

-- default parameters
teleop.parameters = {
}

-- config parameters
teleop:load_parameters()

function teleop:entry()
end

function teleop:update()
end

function teleop:exit()
end

return teleop
