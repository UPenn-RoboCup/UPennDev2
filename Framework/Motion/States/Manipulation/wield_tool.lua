require('Motion_state')
require('Config')

--------------------------------------------------------------------------
-- Wield Tool Controller
--------------------------------------------------------------------------

wield_tool = Motion_state.new('wield_tool')
wield_tool:set_joint_access(0, 'all')
wield_tool:set_joint_access(1, 'upperbody')
wield_tool:set_joint_access(0, 'head')
local dcm = wield_tool.dcm

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
