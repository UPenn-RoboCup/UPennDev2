require('Motion_state')
require('Config')

--------------------------------------------------------------------------
-- Kneel Down Controller
--------------------------------------------------------------------------

kneel_down = Motion_state.new('kneel_down')

local dcm = kneel_down.dcm
local ahrs = Config.ahrs
local joint = Config.joint
kneel_down:set_joint_access(0, joint.all)
kneel_down:set_joint_access(1, joint.lowerbody)

-- default parameters
kneel_down.parameters = {
}

-- config parameters
kneel_down:load_parameters()

function kneel_down:entry()
end

function kneel_down:update()
end

function kneel_down:exit()
end

return kneel_down
