module(..., package.seeall)

function loadconfig(configName)
  local localConfig = require(configName)
  if (type(localConfig) == 'table') then
    for k,v in pairs(localConfig) do
      getfenv()[k] = localConfig[k]
    end
  end
end

loadconfig('Config_RobotisArm_Devices')
loadconfig('Config_RobotisArm_Mechanics')
loadconfig('Config_RobotisArm_Bias')

platform = {}
platform.name = 'RobotisArm'
platform.keyframe_table = 'keyframe_table_RobotisArm'

bias = bias or {}

