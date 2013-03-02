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
  local q = dcm:get_joint_position_sensor()
  local torso_frame = Transform.rotation(pcm:get_torso_rotation())
  local cog = Dynamics.cog(q, torso_frame)

  -- update pcm
  pcm:set_cog(cog)
end

function cog_sensor.exit()
end

return cog_sensor
