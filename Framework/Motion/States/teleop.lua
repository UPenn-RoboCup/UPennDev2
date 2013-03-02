require('Motion_state')

--------------------------------------------------------------------------
-- Tele-Op Controller
--------------------------------------------------------------------------

teleop = Motion_state.new('teleop')
teleop:set_joint_access(0, 'all')
teleop:set_joint_access(1, 'upperbody')
local dcm = teleop.dcm

-- default parameters
teleop.parameters = {
}

function teleop:entry()
end

function teleop:update()
end

function teleop:exit()
end

return teleop
