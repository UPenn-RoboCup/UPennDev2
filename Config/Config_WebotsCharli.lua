module(..., package.seeall)

function loadconfig(configName)
  local localConfig = require(configName)
  if (type(localConfig) == 'table') then
    for k,v in pairs(localConfig) do
      getfenv()[k] = localConfig[k]
    end
  end
end

loadconfig('Config_WebotsCharli_Devices')
loadconfig('Config_WebotsCharli_Bias')
loadconfig('Config_WebotsCharli_Walk')

platform = {}
platform.name = 'WebotsCharli'
platform.keyframe_table = 'keyframe_table_Charli'
platform.walk = 'walkZMP'

walk = getfenv()[platform.walk] or {}
walk.parameters = walk.parameters or {}
walk.increments = walk.increments or {}
walk.a_limits = {0.10, 0.10, 0.10}
walk.v_limits = {0.14, 0.14, 0.20}
