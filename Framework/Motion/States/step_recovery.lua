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
if (Config.motion.step_recovery and Config.motion.step_recovery.parameters) then
  step_recovery:load_parameters(Config.motion.step_recovery.parameters)
end

function step_recovery:entry()
end

function step_recovery:update()
end

function step_recovery:exit()
end

return step_recovery
