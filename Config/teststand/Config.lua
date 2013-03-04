Config = {}

function loadconfig(configName)
  local localConfig = require(configName)
  if (type(localConfig) == 'table') then
    for k,v in pairs(localConfig) do
      Config[k] = localConfig[k]
    end
  end
end

loadconfig('Config_devices')
loadconfig('Config_mechanics')
loadconfig('Config_bias')

Config.platform = {}
Config.platform.name = 'teststand'
Config.platform.keyframe_table = 'keyframe_table_teststand'

return Config
