require('Motion_state')
require('Config')

--------------------------------------------------------------------------
-- Step Recovery Controller
--------------------------------------------------------------------------

step_recovery = Motion_state.new('step_recovery')

local dcm = step_recovery.dcm
local ahrs = Config.ahrs
local joint = Config.joint
step_recovery:set_joint_access(0, joint.all)
step_recovery:set_joint_access(1, joint.lowerbody)

-- default parameters
step_recovery.parameters = {
}

-- config parameters
step_recovery:load_parameters()

function step_recovery:entry()
end

function step_recovery:update()
end

function step_recovery:exit()
end

return step_recovery
