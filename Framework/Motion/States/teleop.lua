require('Motion_state')
require('Config')

--------------------------------------------------------------------------
-- Tele-Op Controller
--------------------------------------------------------------------------

teleop = Motion_state.new('teleop')
teleop:set_joint_access(0, 'all')
teleop:set_joint_access(1, 'upperbody')
local dcm = teleop.dcm

-- default parameters
teleop.parameters = {
}

-- config parameters
if (Config.motion.teleop and Config.motion.teleop.parameters) then
  teleop:load_parameters(Config.motion.teleop.parameters)
end

function teleop:entry()
end

function teleop:update()
end

function teleop:exit()
end

return teleop
