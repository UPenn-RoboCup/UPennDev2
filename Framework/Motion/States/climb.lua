require('Motion_state')
require('Config')

--------------------------------------------------------------------------
-- Climb Controller
--------------------------------------------------------------------------

climb = Motion_state.new('climb')
climb:set_joint_access(0, 'all')
climb:set_joint_access(1, 'lowerbody')
local dcm = climb.dcm

-- default parameters
climb.parameters = {
}

-- config parameters
if (Config.motion.climb and Config.motion.climb.parameters) then
  climb:load_parameters(Config.motion.climb.parameters)
end

function climb:entry()
end

function climb:update()
end

function climb:exit()
end

return climb
