local Config = {}

------------------------
-- General parameters --
------------------------
Config.PLATFORM_NAME = 'THOROP'
Config.nJoint = 35

-----------------------
-- Device Interfaces --
-----------------------
Config.dev = {}
--Config.dev.body         = 'THOROPBody'
Config.dev.body         = 'THOROPBodyUpdate'
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
  -- TODO SJ: Now we just put all config files in /Config folder
  --local exo_name = k..'/Config_'..Config.PLATFORM_NAME..'_'..v
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

-------------
-- DCM Chains --
-------------
-- NOTE: Ignore the MX motors for now
Config.chain = {}
local right_arm = {
  name = 'rarm',
  device = '/dev/ttyUSB0',
  m_ids = {1,3,5,7,9,11,13, --[[head]] 29,30},
  --mx_ids = { 70,65 },
}
local left_arm = {
  name = 'larm',
  device = '/dev/ttyUSB1',
  m_ids = {2,4,6,8,10,12,14,},
  --mx_ids = { 66,67,37, --[[lidar]] },
}
local right_leg = {
  name = 'rleg',
  device = '/dev/ttyUSB2',
  m_ids = {15,17,19,21,23,25, --[[waist pitch]]28},
}
local left_leg = {
  name = 'lleg',
  device = '/dev/ttyUSB3',
  m_ids = {16,18,20,22,24,26, --[[waist yaw]]27}
}
-- Add the one chain support
local one_chain = {
  device = '/dev/ttyUSB0',
  m_ids = {},
}
for _,v in ipairs(right_arm.m_ids) do table.insert(one_chain.m_ids, v) end
for _,v in ipairs(left_arm.m_ids)  do table.insert(one_chain.m_ids, v) end
for _,v in ipairs(right_leg.m_ids) do table.insert(one_chain.m_ids, v) end
for _,v in ipairs(left_leg.m_ids)  do table.insert(one_chain.m_ids, v) end

if OPERATING_SYSTEM=='darwin' then
  right_arm.device = '/dev/cu.usbserial-FTVTLUY0A'
  left_arm.device  = '/dev/cu.usbserial-FTVTLUY0B'
  right_leg.device = '/dev/cu.usbserial-FTVTLUY0C'
  left_leg.device  = '/dev/cu.usbserial-FTVTLUY0D'
  one_chain.device = '/dev/cu.usbserial-FTVTLUY0A'
end
if ONE_CHAIN then
  table.insert(Config.chain, one_chain)
  right_arm = nil
  left_arm  = nil
  right_leg = nil
  left_leg  = nil
else
  table.insert(Config.chain, right_arm)
  table.insert(Config.chain, left_arm)
  table.insert(Config.chain, right_leg)
  table.insert(Config.chain, left_leg)
  one_chain = nil
end

-------------
-- Cameras --
-------------
Config.camera = {}
table.insert(Config.camera,
	{
		name = 'head',
		dev = '/dev/video0',
		fmt = 'yuyv',
		w = 640,
    h = 480,
		fps = 30,
    port = 33333,
    lut = 'lut_nao_new.raw',
    auto_param = {
      {'White Balance, Automatic', 0},
      {'Horizontal Flip', 1},
      {'Vertical Flip', 1},
      {'Auto Exposure', 0},
      {'Auto Exposure Algorithm', 3},
      {'Fade to Black', 0},
      },
    param = {
      {'Brightness', 100},
      {'Contrast', 25},
      {'Saturation', 190},
      {'Hue', 50},
      {'Do White Balance', 100},
      {'Exposure', 35},
      {'Gain', 255},
      {'Sharpness', 0},
      {'Backlight Compensation', 1},
    },
	})

table.insert(Config.camera,
	{
		name = 'hand',
		dev = '/dev/video1',
		fmt = 'yuyv',
		w = 320,
    h = 240,
		fps = 30,
    port = 33334,
    lut = 'lut_nao_new.raw',
    auto_param = {
      {'White Balance, Automatic', 0},
      {'Horizontal Flip', 0},
      {'Vertical Flip', 0},
      {'Auto Exposure', 0},
      {'Auto Exposure Algorithm', 3},
      {'Fade to Black', 0},
      },
    param = {
      {'Brightness', 100},
      {'Contrast', 25},
      {'Saturation', 190},
      {'Hue', 50},
      {'Do White Balance', 100},
      {'Exposure', 35},
      {'Gain', 255},
      {'Sharpness', 0},
      {'Backlight Compensation', 1},
    },
	})

return Config
