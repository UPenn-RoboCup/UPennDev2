require('dcm')
require('pcm')
require('vector')
require('Transform')

--------------------------------------------------------------------------------
-- rotation_sensor : estimates floating base torso pose and twist from AHRS
--------------------------------------------------------------------------------

rotation_sensor = {}

function rotation_sensor.entry()
end

function rotation_sensor.update()
  local euler = dcm:get_ahrs('euler')
  local gyro = dcm:get_ahrs('gyro')

  -- TODO use AHRS orientation data instead of euler angles
  local torso_pose = {0, 0, 0, euler[1], euler[2], 0}
  local torso_twist = {0, 0, 0, gyro[1], gyro[2], gyro[3]}
  local torso_rotation = Transform.pose(torso_pose)

  -- update pcm
  pcm:set_torso_pose(torso_pose)
  pcm:set_torso_twist(torso_twist)
  pcm:set_torso_rotation(torso_rotation)
end

function rotation_sensor.exit()
end

return rotation_sensor
