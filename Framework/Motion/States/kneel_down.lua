require('Motion_state')
require('Config')

--------------------------------------------------------------------------
-- Kneel Down Controller
--------------------------------------------------------------------------

kneel_down = Motion_state.new('kneel_down')
kneel_down:set_joint_access(0, 'all')
kneel_down:set_joint_access(1, 'lowerbody')
local dcm = kneel_down.dcm

-- default parameters
kneel_down.parameters = {
}

-- config parameters
if (Config.motion.kneel_down and Config.motion.kneel_down.parameters) then
  kneel_down:load_parameters(Config.motion.kneel_down.parameters)
end

function kneel_down:entry()
end

function kneel_down:update()
end

function kneel_down:exit()
end

return kneel_down
