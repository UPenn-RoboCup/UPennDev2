require('Motion_state')
require('Config')

--------------------------------------------------------------------------
-- Fall Controller
--------------------------------------------------------------------------

fall = Motion_state.new('fall')

local dcm = fall.dcm
local ahrs = Config.ahrs
local joint = Config.joint
fall:set_joint_access(0, joint.all)
fall:set_joint_access(1, joint.lowerbody)

-- default parameters
fall.parameters = {
}

-- config parameters
fall:load_parameters()

function fall:entry()
end

function fall:update()
end

function fall:exit()
end

return fall
