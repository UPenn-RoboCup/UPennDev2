require('Motion_state')
require('Config')

--------------------------------------------------------------------------
-- Climb Controller
--------------------------------------------------------------------------

climb = Motion_state.new('climb')

local dcm = climb.dcm
local ahrs = Config.ahrs
local joint = Config.joint
climb:set_joint_access(0, joint.all)
climb:set_joint_access(1, joint.lowerbody)

-- default parameters
climb.parameters = {
}

-- config parameters
climb:load_parameters()

function climb:entry()
end

function climb:update()
end

function climb:exit()
end

return climb
