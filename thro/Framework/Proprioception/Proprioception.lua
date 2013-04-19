require('rotation_sensor')
require('contact_sensor')
require('joint_sensor')
require('cog_sensor')
require('cop_sensor')
require('fk_sensor')

--------------------------------------------------------------------------------
-- Proprioception : estimates robot state using proprioceptive sensors
--------------------------------------------------------------------------------

Proprioception = {}

function Proprioception.entry()
  rotation_sensor.entry()
  joint_sensor.entry()
  fk_sensor.entry()
  contact_sensor.entry()
  cop_sensor.entry()
  cog_sensor.entry()
end

function Proprioception.update()
  rotation_sensor.update()
  joint_sensor.update()
  fk_sensor.update()
  contact_sensor.update()
  cop_sensor.update()
  cog_sensor.update()
end

function Proprioception.exit()
  rotation_sensor.exit()
  joint_sensor.exit()
  fk_sensor.exit()
  contact_sensor.exit()
  cop_sensor.exit()
  cog_sensor.exit()
end

return Proprioception
