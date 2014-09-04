Config = {}

------------------------
-- General parameters --
------------------------
Config.PLATFORM_NAME = 'THOROP'
Config.nJoint = 35
-- Dummy arms are the two MX-106R motors per arm
Config.USE_DUMMY_ARMS = false

-----------------------
-- Device Interfaces --
-----------------------
Config.dev = {
	body         = 'THOROPBody',
	game_control = 'OPGameControl',
	team         = 'TeamNSL',
	kick         = 'NewNewKick',
	walk         = 'HumbleWalk',
	crawl        = 'ScrambleCrawl',
	largestep    = 'ZMPStepStair',
	gender       = 'boy',
}

-- Printing of debug messages
Config.debug = {
	webots_wizard=false,	
  -- obstacle = true,
  follow = false,	
  --approach = true,
  --planning = true,
  --goalpost = true,
}

Config.use_angle_localization = true
Config.demo = false
--Config.demo = true
Config.use_localhost = false
--Config.disable_kick = true
Config.disable_kick = false


-- Monitor and logging
Config.enable_monitor = true
Config.enable_log = false
Config.use_log = false

if IS_WEBOTS then
  Config.USE_DUMMY_ARMS = false
  Config.use_gps_pose = false
--  Config.use_gps_pose = true
  
  Config.use_localhost = true
  Config.use_walkkick = true
  --Config.use_walkkick = false
  --Config.backward_approach = true
end

Config.default_role = 2 --0 goalie / 1 attacker / 2 tester
Config.default_state = 5 -- 0 1 2 3 4 for init~finished, 5 for untorqued, 6 for testing

---------------------------
-- Complementary Configs --
---------------------------

Config.torque_legs = true

--SJ: Now we choose the which config to load here
local exo = {
  'Robot',
  'Walk',
  'Net',
  'Manipulation',
  'FSM',
  --'FSM_RoboCup',
  --'FSM_DRCTrials',
  'World',
  'Vision'
}

-- Load each exogenous Config file
for _,v in ipairs(exo) do
	--[[
  --local fname = {HOME, '/Config/Config_', Config.PLATFORM_NAME, '_', v, '.lua'}
  local fname = {'Config/Config_', Config.PLATFORM_NAME, '_', v, '.lua'}
  dofile(table.concat(fname))
	--]]
	----[[
	local fname = {Config.PLATFORM_NAME,'/Config_', Config.PLATFORM_NAME, '_', v}  
  require(table.concat(fname))
	--]]
end




Config.supportY_preview = -0.02
Config.supportY_preview2 = -0.01


return Config
