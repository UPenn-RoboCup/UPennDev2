local Config = {}

------------------------
-- General parameters --
------------------------
Config.PLATFORM_NAME = 'THOROP'

-----------------------
-- Device Interfaces --
-----------------------
Config.dev = {}
Config.dev.body         = Config.PLATFORM_NAME..'Body'
Config.dev.game_control = 'OPGameControl'
Config.dev.team         = 'TeamNSL'
Config.dev.kick         = 'NewNewKick'

--Config.dev.walk         = 'GrumbleWalk'
Config.dev.walk         = 'HumbleWalk'
--Config.dev.walk         = 'CPGWalk'
--Config.dev.walk         = 'ZMPPreviewWalk'
--Config.dev.walk         = 'StaticWalk'

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

---------------------------
-- Complementary Configs --
---------------------------
local exo = {}
exo.Walk = 'Walk'
exo.Net  = 'Net'
exo.FSM  = 'Manipulation' --added

-- Load each exogenous Config file


for k,v in pairs(exo) do
-- SJ:Now we just put all config files in /Config folder
--  local exo_name = k..'/Config_'..Config.PLATFORM_NAME..'_'..v
  local exo_name = '/Config_'..Config.PLATFORM_NAME..'_'..v
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
Config.camera = {}
-- Head
Config.camera.head = {}
Config.camera.head.device = '/dev/video0'
Config.camera.head.resolution = {320,180}
--Config.camera.head.resolution = {640,360}
Config.camera.head.format = 'yuyv'
Config.camera.head.fps = 30
Config.camera.head.quality = 75

-- LWrist (deprecated)
--[[
Config.camera.lwrist = {}
Config.camera.lwrist.device = '/dev/video1'
Config.camera.lwrist.resolution = {320,240}
Config.camera.lwrist.format = 'yuyv'
--Config.camera.lwrist.format = 'mjpeg'
Config.camera.lwrist.fps = 15
Config.camera.lwrist.quality = 75
--]]

return Config