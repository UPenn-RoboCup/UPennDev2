require('Motion_state')
require('Config')

--------------------------------------------------------------------------
-- Kneel Controller
--------------------------------------------------------------------------

kneel = Motion_state.new('kneel')

local dcm = kneel.dcm
local ahrs = Config.ahrs
local joint = Config.joint
kneel:set_joint_access(0, joint.all)
kneel:set_joint_access(1, joint.lowerbody)

-- default parameters
kneel.parameters = {
}

-- config parameters
kneel:load_parameters()

function kneel:entry()
end

function kneel:update()
end

function kneel:exit()
end

return kneel
