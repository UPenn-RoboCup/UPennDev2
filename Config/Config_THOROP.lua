Config = {}

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

-------------
-- Cameras --
-------------
--[[
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
--]]
return Config
