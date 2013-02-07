require('dcm')
require('pcm')
require('vector')
require('Config')
require('Platform')
require('Transform')

--------------------------------------------------------------------------------
-- reference_sensor : estimates torso pose, twist and accel from AHRS
--------------------------------------------------------------------------------

reference_sensor = {}

function reference_sensor.entry()
end

function reference_sensor.update()
  local dt = Platform.get_time_step()
  local torso_pose = vector.zeros(6)
  local torso_twist = vector.zeros(6)
  local torso_accel = vector.zeros(6)

  torso_pose[4], torso_pose[5] = unpack(dcm:get_ahrs('euler'))
  torso_twist[4], torso_twist[5], torso_twist[6] = unpack(dcm:get_ahrs('gyro'))
  torso_accel[1], torso_accel[2], torso_accel[3] = unpack(dcm:get_ahrs('accel'))
  torso_accel[4] = (torso_twist[4] - pcm:get_torso_twist(4))/dt
  torso_accel[5] = (torso_twist[5] - pcm:get_torso_twist(5))/dt
  torso_accel[6] = (torso_twist[6] - pcm:get_torso_twist(6))/dt

  -- TODO use AHRS orientation data instead of euler angles
  local torso_transform = Transform.pose6D(torso_pose)
  local torso_orientation = {}
  for i = 1, 3 do
    for j = 1, 3 do
       torso_orientation[i + (j - 1)*3] = torso_transform[i][j]
    end
  end

  -- update pcm
  pcm:set_torso_pose(torso_pose)
  pcm:set_torso_twist(torso_twist)
  pcm:set_torso_accel(torso_accel)
  pcm:set_torso_orientation(torso_orientation)
end

function reference_sensor.exit()
end

return reference_sensor
