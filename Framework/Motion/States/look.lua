require('Motion_state')

--------------------------------------------------------------------------
-- Look Controller
--------------------------------------------------------------------------

look = Motion_state.new('look')
look:set_joint_access(0, 'all')
look:set_joint_access(1, 'head')
local dcm = look.dcm

-- default parameters
look.parameters = {
}

function look:entry()
end

function look:update()
end

function look:exit()
end

return look
