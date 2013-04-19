require('Motion_state')
require('Config')

--------------------------------------------------------------------------
-- Stand Up Controller
--------------------------------------------------------------------------

stand_up = Motion_state.new('stand_up')

local dcm = stand_up.dcm
local ahrs = Config.ahrs
local joint = Config.joint
stand_up:set_joint_access(0, joint.all)
stand_up:set_joint_access(1, joint.lowerbody)

-- default parameters
stand_up.parameters = {
}

-- config parameters
stand_up:load_parameters()

function stand_up:entry()
end

function stand_up:update()
end

function stand_up:exit()
end

return stand_up
