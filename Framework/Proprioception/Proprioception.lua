require('reference_sensor')
require('contact_sensor')
require('cog_sensor')
require('cop_sensor')
require('fk_sensor')

--------------------------------------------------------------------------------
-- Proprioception : estimates robot state using proprioceptive sensors
--------------------------------------------------------------------------------

Proprioception = {}

function Proprioception.entry()
  reference_sensor.entry()
  fk_sensor.entry()
  contact_sensor.entry()
  cop_sensor.entry()
  cog_sensor.entry()
end

function Proprioception.update()
  reference_sensor.update()
  fk_sensor.update()
  contact_sensor.update()
  cop_sensor.update()
  cog_sensor.update()
end

function Proprioception.exit()
  reference_sensor.exit()
  fk_sensor.exit()
  contact_sensor.exit()
  cop_sensor.exit()
  cog_sensor.exit()
end

return Proprioception
