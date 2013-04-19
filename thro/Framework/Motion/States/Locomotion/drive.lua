require('Motion_state')
require('Config')

--------------------------------------------------------------------------
-- Drive Controller
--------------------------------------------------------------------------

drive = Motion_state.new('drive')

local dcm = drive.dcm
local ahrs = Config.ahrs
local joint = Config.joint
drive:set_joint_access(0, joint.all)
drive:set_joint_access(1, joint.lowerbody)

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
