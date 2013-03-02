require('Motion_state')

--------------------------------------------------------------------------
-- Climb Controller
--------------------------------------------------------------------------

climb = Motion_state.new('climb')
climb:set_joint_access(0, 'all')
climb:set_joint_access(1, 'lowerbody')
local dcm = climb.dcm

-- default parameters
climb.parameters = {
}

function climb:entry()
end

function climb:update()
end

function climb:exit()
end

return climb
