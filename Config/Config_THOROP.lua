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

---------------------------
-- Complementary Configs --
---------------------------
local exo = {}
exo.Walk = 'Walk'
exo.Net  = 'Net'
-- Load each exogenous Config file
for k,v in pairs(exo) do
  local exo_name = k..'/Config_'..Config.PLATFORM_NAME..'_'..v
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