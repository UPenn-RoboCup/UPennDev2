require('Motion_state')

--------------------------------------------------------------------------
-- Climb Off Controller
--------------------------------------------------------------------------

climb_off = Motion_state.new('climb_off')
climb_off:set_joint_access(0, 'all')
climb_off:set_joint_access(1, 'lowerbody')
local dcm = climb_off.dcm

-- default parameters
climb_off.parameters = {
}

function climb_off:entry()
end

function climb_off:update()
end

function climb_off:exit()
end

return climb_off
