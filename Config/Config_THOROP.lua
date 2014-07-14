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
Config.dev.body         = 'THOROPBody'
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

--I hate debug msgs....
Config.debug={
	webots_wizard=false,	
  -- obstacle = true,
  follow = false,
	--approach = false,
  approach = true,
}

-- Dummy arms are the two MX-106R motors per arm
Config.USE_DUMMY_ARMS = true
Config.use_angle_localization = true
Config.demo = false
Config.use_localhost = false
Config.disable_kick = true


if IS_WEBOTS then
  Config.USE_DUMMY_ARMS = false
  Config.use_gps_pose = false
  Config.use_gps_pose = true

  Config.demo = false
  -- Config.demo = true
  
  Config.use_localhost = true
  Config.disable_kick = false
--  Config.disable_kick = true
  Config.use_walkkick = true
--  Config.use_walkkick = false
end





---------------------------
-- Complementary Configs --
---------------------------
local exo = {'Robot', 'Walk', 'Net', 'Manipulation', 'FSM', 'World', 'Vision'}

-- Load each exogenous Config file
for _,v in ipairs(exo) do
  local fname = {HOME, '/Config/Config_', Config.PLATFORM_NAME, '_', v, '.lua'}
  dofile(table.concat(fname))
end

--Vision parameter hack (robot losing ball in webots)
if IS_WEBOTS then
  Config.vision.ball.th_min_fill_rate = 0.25 
  Config.fsm.headLookGoal.yawSweep = 30*math.pi/180
  Config.fsm.headLookGoal.tScan = 2.0

  --for walkkick
  Config.fsm.bodyRobocupApproach.target={0.50,0.12}
  Config.fsm.bodyRobocupApproach.th = {0.10, 0.02}

--Config.backward_approach = true
--[[
  
  Config.fsm.bodyRobocupFollow.circleR = 1.0
  Config.fsm.bodyRobocupFollow.kickoffset = 0.5
--]]

  Config.fsm.bodyRobocupFollow.circleR = 1
  Config.fsm.bodyRobocupFollow.kickoffset = 0.5


--[[
  Config.fsm.bodyRobocupApproach.target={0.25,0.12}  
  Config.fsm.bodyRobocupApproach.th = {0.30, 0.02} 
--]]


  --Even closer!
  Config.fsm.bodyRobocupApproach.target={0.20,0.12}  
  Config.fsm.bodyRobocupApproach.th = {0.25, 0.02}


  Config.world.use_imu_yaw = true
end


Config.torque_legs = true




return Config
