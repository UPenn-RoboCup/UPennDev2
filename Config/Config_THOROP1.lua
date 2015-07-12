--Now the Config file is entirely identical over two robots (using hostname)
IS_STEVE = false
IS_COMPETING = false

if HOSTNAME=="thor-P770ZM" or HOSTNAME=="asus"then	IS_STEVE = false end
-- Global Config
Config = {PLATFORM_NAME = 'THOROP1',demo = false,}
local exo = {'Walk','Net','FSM','World','Vision','Robot_Dale', 'Arm'}

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
}

--NO BIRDWALK FOR ROBOCUP
--Config.birdwalk = 1

Config.raise_body = true
Config.use_exact_tZMP = true
Config.use_heeltoe_walk = true
Config.heeltoe_angle = 0*DEG_TO_RAD
Config.walktraj={}
Config.walktraj.hybridwalk = "foot_trajectory_softfast"
--Config.walktraj.hybridwalk = "foot_trajectory_base"
Config.variable_tstep = true
Config.variable_support = true
Config.arm_init_timeout = true
Config.use_imu_yaw = false --use odometry for yaw





-----------------------------------

-- Tune for Webots
if IS_WEBOTS then
	if IS_STEVE then
		Config.testfile = 'test_teleop'
		Config.debug.armplan = true

	  Config.sensors = {
			ft = true,
      head_camera = 'camera_wizard',
      vision = 'vision_wizard',
      world = 'world_wizard',
			--feedback = 'feedback_wizard',
			--slam = 'slam_wizard',
    	--chest_lidar = true,
    	--head_lidar = true,
			--kinect = 'kinect2_wizard',
			--mesh = 'mesh_wizard',
	  }

	else
		--for SJ's testing in webots
		--Config.testfile = 'test_testbed'
		--Config.testfile = 'test_robocup'
		Config.testfile = 'test_walk_robocup'
		Config.piecewise_step = true
	  Config.sensors = {
			ft = true,
      head_camera = 'camera_wizard',
      vision = 'vision_wizard',
      world = 'world_wizard',

			--feedback = 'feedback_wizard',
	  }
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

Config.use_gps_pose= true


--Vision parameter hack (robot losing ball in webots)
if IS_WEBOTS then

--  Config.vision.ball.th_min_fill_rate = 0.25

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

end



Config.stop_at_neutral = true --false for walk testing
Config.fsm.headTrack.timeout = 3
Config.fsm.dqNeckLimit ={40*DEG_TO_RAD, 180*DEG_TO_RAD}
Config.approachTargetX = {0.45,0.28,0.35} --for first walkkick, long stationary kick, weak walkkick\

if IS_WEBOTS then
	Config.approachTargetX = {
    0.35, --for kick 0 (walkkick)
    0.30, --for kick 1 (st kick)
    0.35  --for kick 2 (weak walkkick)
  }
end

--  Config.approachTargetY= {-0.07,0.05}  --L/R aiming offsets
Config.approachTargetY= {-0.07,0.02}  --L/R aiming offsets
Config.ballX_threshold1 = -1.5 --The threshold we use walkkick
Config.ballX_threshold2 = 0.5 --The threshold we start using strong kick

--Config.torque_legs = false
Config.torque_legs = true
Config.enable_obstacle_scan = true
Config.disable_goal_vision = false

--  Config.auto_state_advance = true
Config.auto_state_advance = false
Config.enable_single_goalpost_detection = true

-- Config.enable_weaker_kick = true
Config.use_walkkick = true
--  Config.use_walkkick = false

Config.disable_ball_when_lookup = true
Config.maxStepApproachTh = 0.30
Config.maxStepApproach1 = 0.10
Config.maxStepApproach2 = 0.06


--final config update
Config.goalieBallX_th = -0.5
Config.goalie_odometry_only = true
Config.goaliePosX = 0.40
Config.ballYFactor = 1.4
Config.gamecontroller_detect = true
Config.gamecontroller_timeout = 5.0
Config.max_goalie_y = 0.7
Config.goalie_threshold_x = 0.10
Config.goalie_t_startmove = 10.0
Config.assume_goalie_blocking = true
Config.enemy_goalie_shift_factor = 0.15

------------------------------------
------------------------------------
------------------------------------
------------------------------------


return Config
