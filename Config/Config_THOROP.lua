local Config = {}

------------------------
-- General parameters --
------------------------
Config.PLATFORM_NAME = 'THOROP'
Config.USE_LOCALHOST = true

-----------------------
-- Device Interfaces --
-----------------------
Config.dev = {}
Config.dev.body         = 'THOROPBody'
Config.dev.game_control = 'OPGameControl'
Config.dev.team         = 'TeamNSL'
Config.dev.kick         = 'NewNewKick'
Config.dev.walk         = 'GrumbleWalk'
Config.dev.crawl        = 'ScrambleCrawl'
Config.dev.largestep    = 'ZMPStepStair'
Config.dev.gender       = 'boy'

--------------------
-- State Machines --
--------------------
Config.fsm = {}
-- Which FSMs should be enabled?
Config.fsm.enabled = {}
if HOME then
  -- Check if include has set some variables
  local unix = require'unix'
  local listing = unix.readdir(HOME..'/Player')
  -- Add all FSM directories that are in Player
  for _,sm in ipairs(listing) do
    if sm:find'FSM' then
      package.path = CWD..'/'..sm..'/?.lua;'..package.path
      table.insert(Config.fsm.enabled,sm)
    end
  end
end

----------------------
-- Network settings --
----------------------
-- TODO: Verify with ifconfig
Config.net = {}
-- Robot IP addresses
Config.net.robot = {
['wired']    = '192.168.123.22',
['wireless'] = '192.168.1.22',
}
-- Remote Operator IP addresses
Config.net.operator = {
['wired']              = '192.168.123.23',
['wired_broadcast']    = '192.168.123.255',
['wireless']           = '192.168.1.23',
['wireless_broadcast'] = '192.168.1.255'
}

-- For use only when testing in webots on a local computer
if Config.USE_LOCALHOST then
  -- wired
  Config.net.robot.wired = 'localhost'
  Config.net.operator.wired = 'localhost'
  Config.net.operator.wired_broadcast = 'localhost'
  -- wireless
  Config.net.robot.wireless = 'localhost'
  Config.net.operator.wireless = 'localhost'
  Config.net.operator.wireless_broadcast = 'localhost'
end

-- Ports
Config.net.reliable_rpc   = 55555
Config.net.unreliable_rpc = 55556
Config.net.team           = 44444
Config.net.state          = 44445
Config.net.head_camera    = 33333
Config.net.mesh           = 33334
Config.net.left_camera    = 33335
Config.net.right_camera   = 33336
Config.net.rgbd           = 33335
Config.net.omap           = 22222
Config.net.hmap           = 22223

---------------------------
-- Complementary Configs --
---------------------------
local exo = {}
exo.Walk = 'Walk'
-- Load each exogenous Config file
for k,v in pairs(exo) do
  local exo_name = k..'/Config_'..Config.PLATFORM_NAME..'_'..v
  print('Loading exogenous',v)
  local exo_config = require(exo_name)
  for kk,vv in pairs(exo_config) do Config[kk] = vv end
end

---------------
-- Keyframes --
---------------
Config.km = {}
Config.km.standup_front = 'km_Charli_StandupFromFront.lua'
Config.km.standup_back  = 'km_Charli_StandupFromBack.lua'

--------------------------
-- Temporary Overwrites --
--------------------------

return Config
