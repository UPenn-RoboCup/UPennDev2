require('Motion_state')
require('Config')

--------------------------------------------------------------------------
-- Turn Wheel Controller
--------------------------------------------------------------------------

turn_wheel = Motion_state.new('turn_wheel')

local dcm = turn_wheel.dcm
local joint = Config.joint
turn_wheel:set_joint_access(0, joint.all)
turn_wheel:set_joint_access(1, joint.upperbody)
turn_wheel:set_joint_access(0, joint.head)

-- default parameters
turn_wheel.parameters = {
}

-- config parameters
turn_wheel:load_parameters()

function turn_wheel:entry()
end

function turn_wheel:update()
end

function turn_wheel:exit()
end

return turn_wheel
