local Config = {}
Config.PLATFORM_NAME = 'THOROP'

-- Exogenuous Configs
local exo = {}
exo.Walk = 'Walk'

for k,v in pairs(exo) do
  local exo_name = k..'/Config_'..Config.PLATFORM_NAME..'_'..v
  print('Loading exogenous',k,v,exo_name)
  local exo_config = require(exo_name)
  for kk,vv in pairs(exo_config) do Config[kk] = vv end
end

-- Device Interface Libraries
Config.dev = {}
Config.dev.body         = 'THOROPBody'
Config.dev.kinematics   = 'THOROPKinematics'
Config.dev.game_control = 'OPGameControl'
Config.dev.team         = 'TeamNSL'
Config.dev.kick         = 'NewNewKick'
Config.dev.walk         = 'GrumbleWalk'
Config.dev.crawl        = 'ScrambleCrawl'
Config.dev.largestep    = 'ZMPStepStair'
Config.dev.gender       = 'boy'

-------------------
-- Network settings
-------------------
-- Remote Operator IP addresses
Config.net = {}
Config.net.operator = {
['wired']     = '192.168.123.23',
['wireless']  = '192.168.1.23',
['wired_broadcast']    = '192.168.123.255',
['wireless_broadcast'] = '192.168.1.255'
}
-- Ports
Config.net.reliable_rpc  = 55555
Config.net.unreliable_rpc  = 55556
Config.net.team = 54321
Config.net.head_camera = 54322
Config.net.mesh = 54322

-- keyframe files
Config.km = {}
Config.km.standup_front = 'km_Charli_StandupFromFront.lua'
Config.km.standup_back = 'km_Charli_StandupFromBack.lua'

return Config