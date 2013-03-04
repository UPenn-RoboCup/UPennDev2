require('Motion_state')
require('Config')

--------------------------------------------------------------------------
-- Kneel Controller
--------------------------------------------------------------------------

kneel = Motion_state.new('kneel')
kneel:set_joint_access(0, 'all')
kneel:set_joint_access(1, 'lowerbody')
local dcm = kneel.dcm

-- default parameters
kneel.parameters = {
}

-- config parameters
if (Config.motion.kneel and Config.motion.kneel.parameters) then
  kneel:load_parameters(Config.motion.kneel.parameters)
end

function kneel:entry()
end

function kneel:update()
end

function kneel:exit()
end

return kneel
