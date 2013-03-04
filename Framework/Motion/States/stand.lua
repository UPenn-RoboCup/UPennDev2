require('Motion_state')
require('Config')

--------------------------------------------------------------------------
-- Stand Controller
--------------------------------------------------------------------------

stand = Motion_state.new('stand')
stand:set_joint_access(0, 'all')
stand:set_joint_access(1, 'lowerbody')
local dcm = stand.dcm

-- default parameters
stand.parameters = {
}

-- config parameters
if (Config.motion.stand and Config.motion.stand.parameters) then
  stand:load_parameters(Config.motion.stand.parameters)
end

function stand:entry()
end

function stand:update()
end

function stand:exit()
end

return stand
