Config = {}

function loadconfig(configName)
  local localConfig = require(configName)
  if (type(localConfig) == 'table') then
    for k,v in pairs(localConfig) do
      Config[k] = localConfig[k]
    end
  end
end

loadconfig('Config_Teststand_Robot')
loadconfig('Config_Teststand_Bias')

Config.platform = {}
Config.platform.name = 'Teststand'
Config.platform.action_table = 'action_table_Teststand'

return Config
