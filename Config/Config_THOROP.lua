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
  --approach = true,
  --planning = true,
}

-- Dummy arms are the two MX-106R motors per arm
Config.USE_DUMMY_ARMS = true
Config.use_angle_localization = true
Config.demo = false
--Config.demo = true
Config.use_localhost = false
--Config.disable_kick = true
Config.disable_kick = false


if IS_WEBOTS then
  Config.USE_DUMMY_ARMS = false
  Config.use_gps_pose = false
--  Config.use_gps_pose = true
  
  Config.use_localhost = true
  Config.use_walkkick = true
  --Config.use_walkkick = false
  --Config.backward_approach = true
end


Config.auto_state_advance = true


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

  Config.fsm.bodyRobocupFollow.circleR = 1
  Config.fsm.bodyRobocupFollow.kickoffset = 0.5


  Config.fsm.bodyRobocupApproach.target={0.25,0.12}  
  Config.fsm.bodyRobocupApproach.th = {0.01, 0.01}
  Config.world.use_imu_yaw = true


  Config.walk.velLimitX = {-.10,.10} 
  Config.walk.velLimitX = {-.10,.15} 
  Config.walk.velLimitY = {-.04,.04}
  Config.walk.velDelta  = {0.04,0.02,0.1}

  Config.stop_after_score = false
  Config.stop_after_score = true
 
end



--FOR real robot
  Config.walk.velLimitX = {-.10,.15} 
  Config.fsm.bodyRobocupApproach.target={0.26,0.14}  
  Config.use_walkkick = true

  Config.torque_legs = true
--Config.torque_legs = false




return Config
