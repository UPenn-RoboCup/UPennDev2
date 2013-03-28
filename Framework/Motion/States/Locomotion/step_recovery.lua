require('Motion_state')
require('Config')

--------------------------------------------------------------------------
-- Step Recovery Controller
--------------------------------------------------------------------------

step_recovery = Motion_state.new('step_recovery')
step_recovery:set_joint_access(0, 'all')
step_recovery:set_joint_access(1, 'lowerbody')
local dcm = step_recovery.dcm

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
