require('Motion_state')
require('Config')

--------------------------------------------------------------------------
-- Idle Controller
--------------------------------------------------------------------------

idle = Motion_state.new('idle')
idle:set_joint_access(0, 'all')
idle:set_joint_access(1, 'lowerbody')
local dcm = idle.dcm

-- default parameters
idle.parameters = {
}

-- config parameters
if (Config.motion.idle and Config.motion.idle.parameters) then
  idle:load_parameters(Config.motion.idle.parameters)
end

function idle:entry()
end

function idle:update()
end

function idle:exit()
end

return idle
