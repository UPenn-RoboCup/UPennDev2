require('Motion_state')
require('Config')

--------------------------------------------------------------------------
-- Look Controller
--------------------------------------------------------------------------

look = Motion_state.new('look')

local dcm = look.dcm
local joint = Config.joint
look:set_joint_access(0, joint.all)
look:set_joint_access(1, joint.head)

-- default parameters
look.parameters = {
}

-- config parameters
look:load_parameters()

function look:entry()
end

function look:update()
end

function look:exit()
end

return look
