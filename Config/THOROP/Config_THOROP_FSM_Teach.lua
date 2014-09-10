assert(Config, 'Need a pre-existing Config table!')

local fsm = {}

-- Update rate in Hz
fsm.update_rate = 100

-- Which FSMs should be enabled?
fsm.enabled = {
  'Body',
  'Motion',
}

--SJ: now we can have multiple FSM options 
fsm.select = {
  Body = 'Teach',
  Motion = 'Teach'
}


fsm.Body = {
  {'bodyIdle', 'init', 'bodyInit'},
  {'bodyInit', 'done', 'bodyStop'},

  {'bodyStop', 'footrace', 'bodyFootRace'},
  {'bodyFootRace', 'done', 'bodyStop'},

  {'bodyStop', 'stepinplace', 'bodyStepPlace'},
--  {'bodyStop', 'stepwaypoint', 'bodyStepWaypoint'},
  {'bodyStop', 'kick', 'bodyRobocupKick'},
  {'bodyStop', 'play', 'bodyRobocupIdle'},
  {'bodyStop', 'goalie', 'bodyRobocupGoalieIdle'},

  {'bodyStop', 'approach', 'bodyRobocupApproach'},

}


fsm.Motion = {
  {'motionIdle', 'timeout', 'motionIdle'},
  {'motionIdle', 'stand', 'motionInit'},
  {'motionIdle', 'bias', 'motionBiasInit'},

  {'motionBiasInit', 'done', 'motionBiasIdle'}, 
  {'motionBiasIdle', 'stand', 'motionInit'}, 

  {'motionInit', 'done', 'motionStance'},

  {'motionStance', 'bias', 'motionBiasInit'},
  {'motionStance', 'preview', 'motionStepPreview'},
  {'motionStance', 'kick', 'motionKick'},
  {'motionStance', 'done_step', 'motionHybridWalkKick'},

  {'motionStance', 'sit', 'motionSit'},
  {'motionSit', 'stand', 'motionStandup'},
  {'motionStandup', 'done', 'motionStance'},

  {'motionStepPreview', 'done', 'motionStance'},
  {'motionKick', 'done', 'motionStance'},

--For new hybrid walk
  {'motionStance', 'hybridwalk', 'motionHybridWalkInit'},
  {'motionHybridWalkInit', 'done', 'motionHybridWalk'},

  {'motionHybridWalk', 'done', 'motionStance'},
  {'motionHybridWalk', 'done', 'motionHybridWalkEnd'},

  {'motionHybridWalk', 'done_step', 'motionHybridWalkKick'},
  {'motionHybridWalkKick', 'done', 'motionStance'},
  {'motionHybridWalkKick', 'walkalong', 'motionHybridWalk'},
  
--  {'motionHybridWalk', 'done_step', 'motionStepNonstop'},
--  {'motionStepNonstop', 'done', 'motionStance'},

  {'motionHybridWalkEnd', 'done', 'motionStance'},

}

Config.fsm = fsm

for _,sm in ipairs(Config.fsm.enabled) do
  if Config.fsm.select[sm] then
    local pname = {HOME, '/Player/', sm, 'FSM/',Config.fsm.select[sm], '/?.lua;', package.path}
    package.path = table.concat(pname)
  else --default fsm
    local pname = {HOME, '/Player/', sm, 'FSM', '/?.lua;', package.path}
    package.path = table.concat(pname)
  end  
end













Config.default_role = 2 --0 goalie / 1 attacker / 2 tester
Config.default_state = 5 -- 0 1 2 3 4 for init~finished, 5 for untorqued, 6 for testing

Config.stop_at_neutral = true --false for walk testing

--Config.fsm.headTrack.timeout = 6  --ORG value
Config.fsm.headTrack.timeout = 3  
Config.fsm.dqNeckLimit ={40*DEG_TO_RAD, 180*DEG_TO_RAD}

Config.approachTargetX = {
  0.45, --for kick 0 (walkkick)
  0.28, --for kick 1 (st kick)
  0.35  --for kick 2 (weak walkkick)
}

--Config.disable_kick = true
Config.disable_kick = false

Config.use_angle_localization = true
Config.demo = false
--Config.demo = true

if IS_WEBOTS then 
  Config.approachTargetX = {
    0.35, --for kick 0 (walkkick)
    0.30, --for kick 1 (st kick)
    0.35  --for kick 2 (weak walkkick)
  }

  Config.fsm.headLookGoal.yawSweep = 30*math.pi/180
  Config.fsm.headLookGoal.tScan = 2.0

  Config.fsm.bodyRobocupFollow.circleR = 1
  Config.fsm.bodyRobocupFollow.kickoffset = 0.5

  Config.fsm.bodyRobocupApproach.target={0.25,0.12}  
  Config.fsm.bodyRobocupApproach.th = {0.01, 0.01}

  Config.stop_after_score = false


--  Config.world.use_imu_yaw = true
--  Config.vision.ball.th_min_fill_rate = 0.25 
  Config.USE_DUMMY_ARMS = false
  Config.use_gps_pose = false
--  Config.use_gps_pose = true
  
  Config.use_localhost = true
  Config.use_walkkick = true
  --Config.use_walkkick = false
  --Config.backward_approach = true
end


--  Config.approachTargetY= {-0.07,0.05}  --L/R aiming offsets
Config.approachTargetY= {-0.07,0.02}  --L/R aiming offsets

Config.ballX_threshold1 = -1.5 --The threshold we use walkkick
Config.ballX_threshold2 = 0.5 --The threshold we start using strong kick

--  Config.use_walkkick = true
Config.use_walkkick = false

--Config.torque_legs = false
Config.torque_legs = true
Config.enable_obstacle_scan = true
Config.disable_goal_vision = false

--  Config.auto_state_advance = true
Config.auto_state_advance = false

Config.enable_single_goalpost_detection = false
Config.enable_single_goalpost_detection = true

-- Config.enable_weaker_kick = true

Config.use_walkkick = true
--  Config.use_walkkick = false

Config.disable_ball_when_lookup = true

Config.maxStepApproachTh = 0.30
Config.maxStepApproach1 = 0.10
Config.maxStepApproach2 = 0.06


--Config.enable_goalie_legspread = true --NOT WORKING FOR NOW
--Config.enable_goalie_legspread = false
--Config.goalie_turn_to_ball = true

---------------------------------------------------------------
--Semi-final config end
---------------------------------------------------------------


Config.goalieBallX_th = -0.5
Config.goalie_odometry_only = true
Config.goaliePosX = 0.40
Config.ballYFactor = 1.4
Config.gamecontroller_detect = true
Config.gamecontroller_timeout = 5.0


Config.max_goalie_y = 0.7
---------------------------------------------------
-- testing

Config.goalie_threshold_x = 0.10
Config.goalie_t_startmove = 10.0


Config.assume_goalie_blocking = true
Config.enemy_goalie_shift_factor = 0.15

return Config
