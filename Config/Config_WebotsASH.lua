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
platform.keyframe_table = 'keyframe_table_ASH'
platform.walk = 'walkOSC'

walk = getfenv()[platform.walk] or {}
walk.parameters = walk.parameters or {}
walk.increments = walk.increments or {}
walk.a_limits = {0.06, 0.06, 0.10}
walk.v_limits = {0.14, 0.14, 0.20}

webots = {}
webots.controlP = 50
webots.maxForce = 1000
webots.maxVelocity = 10
webots.maxAcceleration = -1
