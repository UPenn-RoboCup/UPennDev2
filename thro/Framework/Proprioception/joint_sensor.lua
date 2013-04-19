require('dcm')
require('pcm')

--------------------------------------------------------------------------------
-- joint_sensor : estimates joint state from raw joint data
--------------------------------------------------------------------------------

joint_sensor = {}

function joint_sensor.entry()
end

function joint_sensor.update()
  -- update pcm
  pcm:set_joint_force(dcm:get_joint_force_sensor())
  pcm:set_joint_position(dcm:get_joint_position_sensor())
  pcm:set_joint_velocity(dcm:get_joint_velocity_sensor())
end

function joint_sensor.exit()
end

return joint_sensor
