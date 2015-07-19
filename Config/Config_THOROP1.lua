--Now the Config file is entirely identical over two robots (using hostname)
IS_STEVE = true
IS_COMPETING = false

if HOSTNAME=="thor-P770ZM" or HOSTNAME=="asus"then	IS_STEVE = false end
-- Global Config
Config = {PLATFORM_NAME = 'THOROP1',demo = false,}
local exo = {'Walk','Net','FSM','World','Vision','Robot_Dale', 'Arm', 'Kick'}

--Config = {PLATFORM_NAME = 'THOROP2',demo = false,}
--exo = {'Walk','Net','FSM','World','Vision','Robot_Chip', 'Arm'}

-- Printing of debug messages
Config.debug = {
  webots_wizard = false,
  obstacle = false,
  follow = false,
  approach = false,
  planning = false,
  goalpost = false,
  world = false,
  feedback = false,
  armplan = false,
  walk = false,


  --goalpost = true,
}

Config.arm_init_timeout = true
Config.use_imu_yaw = false --use odometry for yaw

-----------------------------------

-- Tune for Webots
if IS_WEBOTS then

	--for SJ's testing in webots
		--Config.testfile = 'test_testbed'
		--Config.testfile = 'test_robocup'
		Config.testfile = 'test_robocup'
		Config.piecewise_step = true
	  Config.sensors = {
			ft = true,
      head_camera = 'camera_wizard',
--      vision = 'vision_wizard',
      world = 'world_wizard',

			--feedback = 'feedback_wizard',
	  }

	if IS_STEVE then
print('!!!!IS_STEVE!!!!')
		Config.use_gps_pose = false
		Config.use_gps_vision = false

	else
		Config.use_gps_pose = false
		Config.use_gps_vision = false

		Config.use_gps_pose = true
		Config.use_gps_vision = true


	end
end


-----------------------------------
-- Load Paths and Configurations --
-----------------------------------
-- Custom Config files
local pname = {HOME, '/Config/THOROP0/?.lua;', package.path}
package.path = table.concat(pname)
if Config.demo then table.insert(exo, 'Demo') end
for _,v in ipairs(exo) do
	--print('Loading', v)
	local fname = {'Config_THOROP0_', v}
	local filename = table.concat(fname)
  assert(pcall(require, filename))
end

-- Custom motion libraries
for i,sm in pairs(Config.fsm.libraries) do
	local pname = {HOME, '/Player/', i,'/' ,sm, '/?.lua;', package.path}
	package.path = table.concat(pname)
end

-- Finite state machine paths
for sm, en in pairs(Config.fsm.enabled) do
	if en then
		local selected = Config.fsm.select[sm]
		if selected then
			local pname = {HOME, '/Player/', sm, 'FSM/', selected, '/?.lua;', package.path}
			package.path = table.concat(pname)
		else --default fsm
			local pname = {HOME, '/Player/', sm, 'FSM/', '?.lua;', package.path}
			package.path = table.concat(pname)
		end
	end
end


------------------------------------
------------------------------------
------------------------------------
-- ROBOCUP config variables

if IS_WEBOTS then

--  Config.vision.ball.th_min_fill_rate = 0.25

  Config.fsm.headLookGoal.yawSweep = 30*math.pi/180
  Config.fsm.headLookGoal.tScan = 2.0
  Config.fsm.bodyRobocupFollow.circleR = 1
  Config.fsm.bodyRobocupFollow.kickoffset = 0.5
  Config.fsm.bodyRobocupApproach.target={0.25,0.12}
  Config.fsm.bodyRobocupApproach.th = {0.01, 0.01}

  Config.world.use_imu_yaw = true
	Config.world.use_gps_yaw = true
  Config.stop_after_score = true
end

Config.walk.velLimitX = {-.10,.15}
Config.walk.velLimitY = {-.04,.04}
Config.walk.velDelta  = {0.05,0.02,0.1}
  


------------------------------------------------------------
-- Head/vision parameters
Config.use_angle_localization = true
Config.fsm.headTrack.timeout = 3
Config.fsm.dqNeckLimit ={40*DEG_TO_RAD, 180*DEG_TO_RAD}

Config.enable_obstacle_scan = true
Config.disable_goal_vision = false
Config.auto_state_advance = false
Config.enable_single_goalpost_detection = true


Config.disable_ball_when_lookup = true
------------------------------------------------------------





Config.stop_at_neutral = true --false for walk testing


----------------------------------------------------
--Approach parameters

Config.approachTargetX = {0.55,0.40,0.55} --for first walkkick, long stationary kick, weak walkkick\
Config.approachTargetY= {0.00,0.00}  --L/R aiming offsets
if IS_WEBOTS then
	Config.approachTargetX = {0.35, 0.30, 0.35}  --for first walkkick, long stationary kick, weak walkkick
	Config.approachTargetY= {-0.0,0.0}  --L/R aiming offsets (for robot pose)
end

Config.ballX_threshold1 = -1.5 --The threshold we use walkkick
Config.ballX_threshold2 = 0.5 --The threshold we start using strong kick


--for grass, we keep using walkkick until very close
Config.ballX_threshold1 = 1.0 --The threshold we use walkkick
Config.ballX_threshold2 = 2.5 --The threshold we start using strong kick


Config.maxStepApproachTh = 0.30
Config.maxStepApproach1 = 0.10
Config.maxStepApproach2 = 0.06


Config.assume_goalie_blocking = true
Config.enemy_goalie_shift_factor = 0.15


--faster approach
Config.maxStepApproach1 = 0.15
Config.maxStepApproach2 = 0.10



----------------------------------------------------





----------------------------------------------------
-- Goalie bahavior
Config.goalieBallX_th = -0.5
Config.goalie_odometry_only = true
Config.goaliePosX = 0.40
Config.ballYFactor = 1.4
Config.max_goalie_y = 0.7
Config.goalie_threshold_x = 0.10
Config.goalie_t_startmove = 10.0
------------------------------------

Config.gamecontroller_detect = true
Config.gamecontroller_timeout = 5.0


Config.demo = true
--Config.disable_kick = true --use this for approach testing
Config.use_walkkick = true


Config.driftFactor = {0,0,0}



return Config
