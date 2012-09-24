module(..., package.seeall)

function loadconfig(configName)
  local localConfig = require(configName)
  if (type(localConfig) == 'table') then
    for k,v in pairs(localConfig) do
      getfenv()[k] = localConfig[k]
    end
  end
end

loadconfig('Config_ASH_Devices')
loadconfig('Config_ASH_Bias')
loadconfig('Config_ASH_Walk')

platform = {}
platform.name = 'ASH'
platform.keyframe_table = 'keyframe_table_ASH'
platform.walk = 'walkOSC'

walk = getfenv()[platform.walk] or {}
walk.parameters = walk.parameters or {}
walk.increments = walk.increments or {}
walk.a_limits = {0.04, 0.04, 0.10}
walk.v_limits = {0.08, 0.08, 0.10}
