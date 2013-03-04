module(..., package.seeall)

function loadconfig(configName)
  local localConfig = require(configName)
  if (type(localConfig) == 'table') then
    for k,v in pairs(localConfig) do
      getfenv()[k] = localConfig[k]
    end
  end
end

loadconfig('Config_devices')
loadconfig('Config_mechanics')
loadconfig('Config_bias')

platform = {}
platform.name = 'robotis_arm'
platform.keyframe_table = 'keyframe_table_robotis_arm'

bias = bias or {}

