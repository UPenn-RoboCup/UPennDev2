assert(Config, 'Need a pre-existing Config table!')

local fsm = {}

-- Do we disable FSMs?
fsm.disabled = false

-- Do we disable Kick?
--fsm.disable_kick = true
fsm.disable_kick = false

-- Update rate in Hz
fsm.update_rate = 100

-- Which FSMs should be enabled?
fsm.enabled = {
  'Arm',
  'Body',
  'Head',
  'Motion',
}

--SJ: now we can have multiple FSM options 
fsm.select = {
  Arm = 'Default',  
  Head = 'RoboCup',
  Body = 'RoboCup',
  Motion = 'RoboCup'
}


fsm.Arm = {
  {'armIdle', 'timeout', 'armIdle'},
  {'armIdle', 'init', 'armInit'},
  {'armInit', 'done', 'armPose1'},
}

fsm.Head = {
  {'headIdle', 'scan', 'headBackScan'},
  {'headIdle', 'teleop', 'headTeleop'},
  {'headIdle', 'scanobs', 'headObstacleScan'},
  --

  {'headScan', 'ballfound', 'headTrack'},
  {'headScan', 'noball', 'headBackScan'},
  {'headScan', 'teleop', 'headTeleop'},
  {'headScan', 'scanobs', 'headObstacleScan'},
  -- 
  {'headTeleop', 'scan', 'headBackScan'},
  {'headTeleop', 'scanobs', 'headObstacleScan'},
  -- 
  
  {'headObstacleScan', 'done', 'headTrack'},
  {'headObstacleScan', 'backscan', 'headBackScan'},
  {'headObstacleScan', 'teleop', 'headTeleop'},
 

  {'headBackScan', 'ballfound', 'headTrack'},
  {'headBackScan', 'noball', 'headBackScan'},
  {'headBackScan', 'teleop', 'headTeleop'},
  {'headBackScan', 'scanobs', 'headObstacleScan'},
  
  --
  {'headTrack', 'balllost', 'headScan'},
  {'headTrack', 'timeout', 'headLookGoal'},
  {'headTrack', 'teleop', 'headTeleop'},
  {'headTrack', 'scanobs', 'headObstacleScan'},
  {'headTrack', 'kickfollow', 'headKickFollow'},
  --
  {'headKickFollow', 'done', 'headTrack'},

  {'headLookGoal', 'timeout', 'headTrack'},
  {'headLookGoal', 'scanobs', 'headObstacleScan'},
  --
  --
  
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

--test stuff
  {'bodyStepPlace',   'done', 'bodyStop'},
--  {'bodyStepWaypoint',   'done', 'bodyStop'},

  -------------------------------
  --ATTACKER SMs-----------------------------
  -------------------------------
  
  {'bodyRobocupIdle', 'timeout', 'bodyRobocupIdle'},
  {'bodyRobocupIdle', 'ballfound', 'bodyRobocupFollow'},
  {'bodyRobocupIdle','stop','bodyStop'},
  {'bodyRobocupIdle','goalie','bodyRobocupGoalieIdle'},

  {'bodyRobocupFollow', 'done', 'bodyRobocupIdle'},
  {'bodyRobocupFollow', 'timeout', 'bodyRobocupFollow'},
  {'bodyRobocupFollow', 'ballclose', 'bodyRobocupApproach'},
  {'bodyRobocupFollow','stop','bodyStop'},
  
  {'bodyRobocupApproach', 'done', 'bodyRobocupKick'},
  {'bodyRobocupApproach', 'ballfar', 'bodyRobocupFollow'},
  {'bodyRobocupApproach','stop','bodyStop'},
--  {'bodyRobocupApproach', 'done', 'bodyStop'}, --we just stop in front of the ball to test code

  {'bodyRobocupKick', 'done', 'bodyRobocupIdle'},
  {'bodyRobocupKick', 'testdone', 'bodyStop'},


  -------------------------------
  --Goalie SMs-----------------------------
  -------------------------------

  {'bodyRobocupGoalieIdle', 'attacker', 'bodyRobocupIdle'},
  {'bodyRobocupGoalieIdle', 'timeout', 'bodyRobocupGoalieIdle'},
  {'bodyRobocupGoalieIdle', 'ballfound', 'bodyRobocupGoalieAnticipate'},

  {'bodyRobocupGoalieAnticipate', 'timeout', 'bodyRobocupGoalieAnticipate'},
  {'bodyRobocupGoalieAnticipate', 'reposition', 'bodyRobocupGoalieReposition'},
  {'bodyRobocupGoalieAnticipate', 'stop', 'bodyStop'},


  {'bodyRobocupGoalieReposition', 'done', 'bodyRobocupGoalieAnticipate'},
  {'bodyRobocupGoalieReposition', 'stop', 'bodyStop'},


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

fsm.dqNeckLimit = {
  60 * DEG_TO_RAD, 60 * DEG_TO_RAD
}

fsm.headScan = {
  pitch0 = 30 * DEG_TO_RAD,
  pitchMag = 20 * DEG_TO_RAD,
  --yawMag = 80 * DEG_TO_RAD,
  yawMag = 40 * DEG_TO_RAD,
  tScan = 5, --sec
}

--HeadReady
fsm.headReady = {
  dist = 3
}

--HeadTrack
fsm.headTrack = {
  tLost = 5,
  timeout = 6,
	dist_th = 0.5,
}

--HeadLookGoal: Look up to see the goal
fsm.headLookGoal = {
  yawSweep = 80*DEG_TO_RAD,
}

--HeadSweep: Look around to find the goal
fsm.headSweep = {
  tScan = 2.0,
  tWait = 0.25,
}

fsm.headObstacleScan = {
  yawMag = 55*DEG_TO_RAD,
  pitchUp = 25*DEG_TO_RAD,
  pitchDown = 35*DEG_TO_RAD,
}

fsm.bodyRobocupFollow = {
  th_lfoot = 0.001,
  th_rfoot = 0.001,
  th_dist = 0.08,  --TODO
}

fsm.bodyRobocupApproach = {
  target={0.30,0.12} ,
  th = {0.34, 0.02}
}

if IS_WEBOTS then
  fsm.headScan.tScan = 16
  fsm.bodyRobocupFollow.th_dist = 0.2
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

Config.fsm = fsm

return Config
