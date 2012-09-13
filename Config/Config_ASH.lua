module(..., package.seeall)

function loadconfig(configName)
  local localConfig = require(configName)
  if (type(localConfig) == 'table') then
    for k,v in pairs(localConfig) do
      getfenv()[k] = localConfig[k]
    end
  end
end

loadconfig('Config_ASH_Robot')
loadconfig('Config_ASH_Walk')
loadconfig('Config_ASH_Bias')

platform = {}
platform.name = 'ASH'
platform.action_table = 'action_table_ASH'
platform.walk = 'walkOSC'

walk = getfenv()[platform.walk] or {}
walk.parameters = walk.parameters or {}
walk.increments = walk.increments or {}
