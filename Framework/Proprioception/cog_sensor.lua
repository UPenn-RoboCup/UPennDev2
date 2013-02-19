require('dcm')
require('pcm')
require('vector')
require('Config')
require('Dynamics')
require('Transform')

--------------------------------------------------------------------------------
-- cog_sensor : estimates center of gravity from current joint positions 
--------------------------------------------------------------------------------

cog_sensor = {}

function cog_sensor.entry()
end

function cog_sensor.update()
  -- TODO use torso_orientation instead of torso_pose
  local q = dcm:get_joint_position_sensor()
  local torso_pose = pcm:get_torso_pose()
  local torso_frame = Transform.pose6D(torso_pose)
  local cog = Dynamics.cog(q, torso_frame)

  pcm:set_cog(cog)
end

function cog_sensor.exit()
end

return cog_sensor
