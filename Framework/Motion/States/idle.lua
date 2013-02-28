require('Motion_state')

--------------------------------------------------------------------------
-- Idle Controller
--------------------------------------------------------------------------

idle = Motion_state.new('idle')
idle:set_joint_access(0, 'all')
idle:set_joint_access(1, 'lowerbody')
local dcm = idle.dcm

-- default parameters
idle.parameters = {
}

function idle:entry()
end

function idle:update()
end

function idle:exit()
end

return idle
