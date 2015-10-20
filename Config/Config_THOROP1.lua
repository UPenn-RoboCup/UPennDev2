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


		Config.testfile = 'test_testbed'

		Config.piecewise_step = true
	  Config.sensors = {
			ft = true,
      head_camera = 'camera_wizard',
      vision = 'vision_wizard',
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

	--Config.use_gps_pose = true
--		Config.use_gps_vision = true


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

Config.reject_forward_balls = true
Config.dont_look_goals =true
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

Config.maxStepApproach1 = 0.15
Config.maxStepApproach2 = 0.10
----------------------------------------------------


----------------------------------------------------
-- Goalie bahavior
Config.goalieBallX_th = -0.5
Config.goalie_odometry_only = false
Config.goaliePosX = 0.40
Config.ballYFactor = 1.4
Config.max_goalie_y = 0.7
Config.goalie_threshold_x = 0.10
Config.goalie_t_startmove = 10.0
------------------------------------

Config.gamecontroller_detect = true
Config.gamecontroller_timeout = 5.0


--Config.disable_kick = true --use this for approach testing
Config.use_walkkick = true
Config.use_arm_switch = true


  --low/med/fast speed drift (per step)
if IS_WEBOTS then
	Config.driftFactor = {0,0,0}
else
	Config.driftFactor = {0.029,0.026,0.008}
end

Config.spin_detect = true
Config.velThApproach = {0.02,0.02}
Config.approachTargetY= {-0.08,0.08}  --L/R aiming offsets (kick with outside of the foot!)

--We use walkkick -> weak walkkick -> strong kick in sequence
Config.ballX_threshold1 = 0.0--The threshold we use walkkick
Config.ballX_threshold2 = 1.0 --The threshold we start using strong kick
------------------------------------------------------------------------------------------------



-- DAY 2
Config.approachTargetY= {-0.08,0.10}  --L/R aiming offsets (kick with outside of the foot!)
if not IS_WEBOTS then
	Config.driftFactor = {0.029,0.026,0.020}
	Config.approachTargetY= {-0.02,0.06}  --L/R aiming offsets (kick with outside of the foot!)
	Config.approachTargetX = {
	--0.40, --basic walkkick
	0.48, --basic walkkick
	--0.34, --long stationary kick
	0.30, --long stationary kick
	0.40} --for first walkkick, long stationary kick, weak walkkick\
end

Config.ballX_threshold_direct = 0 --after halfline, we just aim for the goal


Config.dont_look_goals =false --testing
Config.auto_init = true
Config.bodymove_fix = true







--[[
--testing w/o vision
-------------------------------------
      Config.sensors = {
			ft = true,
      head_camera = 'camera_wizard',
      world = 'world_wizard',
	  }

		Config.use_gps_pose = true
		Config.use_gps_vision = true
--]]
--Config.debug.planning=true


--DAY 3
Config.assume_goalie_blocking = true
Config.assume_goalie_blocking = false
Config.kick_decision_new = true
Config.kick_threshold= {4.5,3.5}
Config.obsDistTh = 1.0 --larger obstacle distance th
Config.borderTh = 0.6
Config.ballX_threshold_direct = 0
Config.new_planner = true


Config.goalie_avoid_kick = 0 --dead center
--Config.goalie_avoid_kick = 0.5 --1/2 between center and the post
--Config.goalie_avoid_kick = 1

--SEMI FINAL
Config.aim_far_goalpost = false
Config.aim_far_goalpost = true
Config.goalie_avoid_kick = 0.5
Config.farpost_aim_th = 3 --closer than this, aim for farther post



--FINAL
Config.goalie_spread_enable = true
Config.ballX_th_goalie_movement = -2 --penalty mark
Config.doublecheck_stance = true --check foot stance before walkkick


--[[
Config.walk.phSingle = {0.1,0.9}
Config.walk.phZmp = {0.05,0.95}


Config.walk.phSingle = {0.1,0.9}
Config.walk.phZmp = {0.02,0.98}
--]]

Config.walk.velLimitX = {-.10,.60}
Config.walk.stanceLimitX = {-0.60,0.60}

Config.walk.phSingle = {0.2,0.8}
Config.walk.phZmp = {0.2,0.8}



--MOVING ZMP
Config.walk.heeltoe_vel_min = 0.19 --MIN speed for heeltoe walk

Config.walk.zmpHeel = -0.05
Config.walk.zmpToe = 0.05

Config.use_heeltoe_walk=true
--Config.walk.zmpHeel,Config.walk.zmpToe = 0,0
Config.walk.zmpHeel,Config.walk.zmpToe = -0.0,0.10


Config.walk.heel_angle = 5*DEG_TO_RAD
Config.walk.toe_angle = 5*DEG_TO_RAD

Config.walk.heel_angle = 10*DEG_TO_RAD
Config.walk.toe_angle = 10*DEG_TO_RAD



--[[
Config.walk.heel_angle = 20*DEG_TO_RAD
Config.walk.toe_angle = 20*DEG_TO_RAD
Config.walk.stepHeight = 0.07 --test--]]


Config.walk.heeltoe_vel_min = 0.419 --MIN speed for heeltoe walk

return Config
