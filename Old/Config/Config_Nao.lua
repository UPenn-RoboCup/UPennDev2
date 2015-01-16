Config = {}

------------------------
-- General parameters --
------------------------
Config.PLATFORM_NAME = 'Nao'
Config.nJoint = 35

-----------------------
-- Device Interfaces --
-----------------------
Config.dev = {}
Config.dev.gender       = 'boy'
Config.dev.body         = 'NaoBody'
Config.dev.game_control = 'OPGameControl'
Config.dev.team         = 'TeamNSL'
Config.dev.kick         = 'ZMPStepKick'
Config.dev.walk         = 'CleanWalk'

-- OLD SUPPORT
Config.dev.kinematics = 'NaoKinematics'

--------------------
-- State Machines --
--------------------
Config.fsm = {}
-- Which FSMs should be enabled?
Config.fsm.enabled = {}

---------------------------
-- Complementary Configs --
---------------------------
local exo = {}
exo.Walk = 'Walk'
exo.Net = 'Net'
exo.fsm = 'FSM'

-- Load each exogenous Config file
for k,v in pairs(exo) do
  --local exo_name = '/Config_'..Config.PLATFORM_NAME..'_'..v
  local exo_name = {Config.PLATFORM_NAME,'/Config_', Config.PLATFORM_NAME, '_', v}  

  local exo_config = require(exo_name)
  for kk,vv in pairs(exo_config) do Config[kk] = vv end
end

-- Override legacy...
-- Which FSMs should be enabled?
Config.fsm.enabled = {}

---------------
-- Keyframes --
---------------
Config.km = {}
Config.km.standup_front = 'km_WebotsNao_StandupFromFront.lua'
Config.km.standup_back  = 'km_WebotsNao_StandupFromBack.lua'

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
    lut = 'lut_NaoV4_Grasp.raw',
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
    lut = 'lut_NaoV4_Grasp.raw',
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
  
------------
-- Vision --
------------
Config.vision = {}
-- Vision for camera 1
table.insert(Config.vision,{
  focal_length = 545.6,
  focal_base = 640,
  scaleA = 2,
  scaleB = 2,
  -- NOTE: Should verify with camera config...
	w = 320,
  h = 240,
  bodyHeight = 0.315,
  bodyTilt = 0,
  footX = 0,
  neckX = 0,
  neckZ = 0.14,
  pCamera = {0.05871, 0.0, 0.06364},
  pitchCamera = 1.2*math.pi/180,
  -- If the ball key, then search for ball
  ball = {
    max_distance = 5.0,
    max_height = 0.20,
    diameter = 0.065,
  }
  })
-- Vision for camera 2
table.insert(Config.vision,{
  focal_length = 545.6,
  focal_base = 640,
  scaleA = 2,
  scaleB = 2,
  -- NOTE: Should verify with camera config...
	w = 320,
  h = 240,
  bodyHeight = 0.315,
  bodyTilt = 0,
  footX = 0,
  neckX = 0,
  neckZ = 0.14,
  pCamera = {0.05071, 0.0, 0.01774},
  pitchCamera = 39.7*math.pi/180,
  ball = {
    max_distance = 5.0,
    max_height = 0.20,
    diameter = 0.065,
  }
  })
  
-- Legacy...
local head = {}

head.camOffsetZ = 0.41;
head.pitchMin = -35*math.pi/180;
head.pitchMax = 30*math.pi/180;
head.yawMin = -120*math.pi/180;
head.yawMax = 120*math.pi/180;
--Update with naoV4 camera values
head.cameraPos = {{0.05871, 0.0, 0.06364},
                  {0.05071, 0.0, 0.01774}}; 
head.cameraAngle = {{0.0, 1.2*math.pi/180, 0.0},
                    {0.0, 39.7*math.pi/180, 0.0}};

head.neckZ=0.14; --From CoM to neck joint
head.neckX=0;  
head.bodyTilt = 0;
Config.head = head

return Config
