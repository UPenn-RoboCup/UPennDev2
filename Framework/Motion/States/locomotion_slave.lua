require('Motion_state')
require('Config')

--------------------------------------------------------------------------
-- Locomotion Slave Controller
--------------------------------------------------------------------------

locomotion_slave = Motion_state.new('locomotion_slave')
locomotion_slave:set_joint_access(0, 'all')
locomotion_slave:set_joint_access(1, 'upperbody')
local dcm = locomotion_slave.dcm

-- default parameters
locomotion_slave.parameters = {
}

-- config parameters
if (Config.motion.locomotion_slave and Config.motion.locomotion_slave.parameters) then
  locomotion_slave:load_parameters(Config.motion.locomotion_slave.parameters)
end

function locomotion_slave:entry()
end

function locomotion_slave:update()
end

function locomotion_slave:exit()
end

return locomotion_slave
