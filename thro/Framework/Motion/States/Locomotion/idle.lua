require('Motion_state')
require('Config')

--------------------------------------------------------------------------
-- Idle Controller
--------------------------------------------------------------------------

idle = Motion_state.new('idle')

local dcm = idle.dcm
local ahrs = Config.ahrs
local joint = Config.joint
idle:set_joint_access(0, joint.all)
idle:set_joint_access(1, joint.lowerbody)

-- default parameters
idle.parameters = {
}

-- config parameters
idle:load_parameters()

function idle:entry()
end

function idle:update()
end

function idle:exit()
end

return idle
