Config = {
  PLATFORM_NAME = 'THOROP',
  nJoint = 35,
}

-- Interfaces
Config.dev = {
  body = 'THOROPBody',
  game_control = 'OPGameControl',
  team = 'TeamNSL',
  kick = 'NewNewKick',
  walk = 'HumbleWalk', -- 'ZMPPreviewWalk'
  crawl = 'ScrambleCrawl',
  largestep = 'ZMPStepStair',
  gender = 'boy',
}

---------------------------
-- Complementary Configs --
---------------------------
local exo = {'Robot', 'Walk', 'Net', 'Manipulation', 'FSM', 'World', 'Vision'}

-- Load each exogenous Config file
for _,v in ipairs(exo) do
  local fname = {HOME, '/Config/Config_', Config.PLATFORM_NAME, '_', v, '.lua'}
  dofile(table.concat(fname))
end

---------------
-- Keyframes --
---------------
Config.km = {}
Config.km.standup_front = 'km_Charli_StandupFromFront.lua'
Config.km.standup_back  = 'km_Charli_StandupFromBack.lua'

return Config
