require('Motion_state')
require('Config')

--------------------------------------------------------------------------
-- Scan Controller
--------------------------------------------------------------------------

scan = Motion_state.new('scan')
scan:set_joint_access(0, 'all')
scan:set_joint_access(1, 'head')
local dcm = scan.dcm

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
