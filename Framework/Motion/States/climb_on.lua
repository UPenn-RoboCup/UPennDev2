require('Motion_state')
require('Config')

--------------------------------------------------------------------------
-- Climb On Controller
--------------------------------------------------------------------------

climb_on = Motion_state.new('climb_on')
climb_on:set_joint_access(0, 'all')
climb_on:set_joint_access(1, 'lowerbody')
local dcm = climb_on.dcm

-- default parameters
climb_on.parameters = {
}

-- config parameters
if (Config.motion.climb_on and Config.motion.climb_on.parameters) then
  climb_on:load_parameters(Config.motion.climb_on.parameters)
end

function climb_on:entry()
end

function climb_on:update()
end

function climb_on:exit()
end

return climb_on
