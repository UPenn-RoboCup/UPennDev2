require('Motion_state')

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

function locomotion_slave:entry()
end

function locomotion_slave:update()
end

function locomotion_slave:exit()
end

return locomotion_slave
