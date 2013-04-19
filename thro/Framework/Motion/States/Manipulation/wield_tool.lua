require('Motion_state')
require('Config')

--------------------------------------------------------------------------
-- Wield Tool Controller
--------------------------------------------------------------------------

wield_tool = Motion_state.new('wield_tool')

local dcm = wield_tool.dcm
local joint = Config.joint
wield_tool:set_joint_access(0, joint.all)
wield_tool:set_joint_access(1, joint.upperbody)
wield_tool:set_joint_access(0, joint.head)

-- default parameters
wield_tool.parameters = {
}

-- config parameters
wield_tool:load_parameters()

function wield_tool:entry()
end

function wield_tool:update()
end

function wield_tool:exit()
end

return wield_tool
