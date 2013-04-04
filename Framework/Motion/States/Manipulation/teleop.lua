require('Motion_state')
require('Config')

--------------------------------------------------------------------------
-- Tele-Op Controller
--------------------------------------------------------------------------

teleop = Motion_state.new('teleop')

local dcm = teleop.dcm
local joint = Config.joint
teleop:set_joint_access(0, joint.all)
teleop:set_joint_access(1, joint.upperbody)
teleop:set_joint_access(0, joint.head)

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
