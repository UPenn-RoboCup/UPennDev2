module(..., package.seeall)

function loadconfig(configName)
  local localConfig = require(configName)
  if (type(localConfig) == 'table') then
    for k,v in pairs(localConfig) do
      getfenv()[k] = localConfig[k]
    end
  end
end

loadconfig('Config_WebotsASH_Robot')
loadconfig('Config_WebotsASH_Walk')
loadconfig('Config_WebotsASH_Bias')

platform = {}
platform.name = 'WebotsASH'
platform.action_table = 'action_table_ASH'
platform.walk = 'walkZMP'

walk = getfenv()[platform.walk] or {}

webots = {}
webots.controlP = 50
webots.maxForce = 1000
webots.maxVelocity = 10
webots.maxAcceleration = -1
