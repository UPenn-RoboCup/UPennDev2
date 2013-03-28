require('Motion_state')
require('Config')

--------------------------------------------------------------------------
-- Drive Controller
--------------------------------------------------------------------------

drive = Motion_state.new('drive')
drive:set_joint_access(0, 'all')
drive:set_joint_access(1, 'lowerbody')
local dcm = drive.dcm

-- default parameters
drive.parameters = {
}

-- config parameters
drive:load_parameters()

function drive:entry()
end

function drive:update()
end

function drive:exit()
end

return drive
