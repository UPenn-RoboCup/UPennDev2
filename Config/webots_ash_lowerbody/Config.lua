Config = {}

function loadconfig(configName)
  local localConfig = require(configName)
  if (type(localConfig) == 'table') then
    for k,v in pairs(localConfig) do
      Config[k] = localConfig[k]
    end
  end
end

loadconfig('Config_bias')
loadconfig('Config_devices')
loadconfig('Config_mechanics')
loadconfig('Config_motion')

Config.platform = {}
Config.platform.name = 'webots_ash_lowerbody'

return Config
