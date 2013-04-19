require('Motion_state')
require('Config')

--------------------------------------------------------------------------
-- Climb Off Controller
--------------------------------------------------------------------------

climb_off = Motion_state.new('climb_off')

local dcm = climb_off.dcm
local ahrs = Config.ahrs
local joint = Config.joint
climb_off:set_joint_access(0, joint.all)
climb_off:set_joint_access(1, joint.lowerbody)

-- default parameters
climb_off.parameters = {
}

-- config parameters
climb_off:load_parameters()

function climb_off:entry()
end

function climb_off:update()
end

function climb_off:exit()
end

return climb_off
