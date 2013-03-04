require('Motion_state')
require('Config')

--------------------------------------------------------------------------
-- Climb Off Controller
--------------------------------------------------------------------------

climb_off = Motion_state.new('climb_off')
climb_off:set_joint_access(0, 'all')
climb_off:set_joint_access(1, 'lowerbody')
local dcm = climb_off.dcm

-- default parameters
climb_off.parameters = {
}

-- config parameters
if (Config.motion.climb_off and Config.motion.climb_off.parameters) then
  climb_off:load_parameters(Config.motion.climb_off.parameters)
end

function climb_off:entry()
end

function climb_off:update()
end

function climb_off:exit()
end

return climb_off
