require('Motion_state')
require('Config')

--------------------------------------------------------------------------
-- Scan Controller
--------------------------------------------------------------------------

scan = Motion_state.new('scan')

local dcm = scan.dcm
local joint = Config.joint
scan:set_joint_access(0, joint.all)
scan:set_joint_access(1, joint.head)

-- default parameters
scan.parameters = {
}

-- config parameters
scan:load_parameters()

function scan:entry()
end

function scan:update()
end

function scan:exit()
end

return scan
