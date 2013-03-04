require('Motion_state')
require('Config')

--------------------------------------------------------------------------
-- Stand Up Controller
--------------------------------------------------------------------------

stand_up = Motion_state.new('stand_up')
stand_up:set_joint_access(0, 'all')
stand_up:set_joint_access(1, 'lowerbody')
local dcm = stand_up.dcm

-- default parameters
stand_up.parameters = {
}

-- config parameters
if (Config.motion.stand_up and Config.motion.stand_up.parameters) then
  stand_up:load_parameters(Config.motion.stand_up.parameters)
end

function stand_up:entry()
end

function stand_up:update()
end

function stand_up:exit()
end

return stand_up
