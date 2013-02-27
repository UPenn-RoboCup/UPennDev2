require('Motion_state')

--------------------------------------------------------------------------
-- Stand Controller
--------------------------------------------------------------------------

stand = Motion_state.new('stand')
stand:set_joint_access(0, 'all')
stand:set_joint_access(1, 'lowerbody')
local dcm = stand.dcm

-- default parameters
stand.parameters = {
}

function stand:entry()
end

function stand:update()
end

function stand:exit()
end

return stand
