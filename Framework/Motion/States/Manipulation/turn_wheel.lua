require('Motion_state')
require('Config')

--------------------------------------------------------------------------
-- Turn Wheel Controller
--------------------------------------------------------------------------

turn_wheel = Motion_state.new('turn_wheel')
turn_wheel:set_joint_access(0, 'all')
turn_wheel:set_joint_access(1, 'upperbody')
turn_wheel:set_joint_access(0, 'head')
local dcm = turn_wheel.dcm

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
