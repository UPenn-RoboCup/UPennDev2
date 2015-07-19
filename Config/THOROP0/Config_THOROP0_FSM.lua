assert(Config, 'Need a pre-existing Config table!')

local fsm = {}

Config.torque_legs = true

-- Update rate in Hz
fsm.update_rate = 120
-- TODO: Grab from Webots, too...

-- Which FSMs should be enabled?
fsm.enabled = {
	Arm = true,
	Body = true,
	Head = true,
	Motion = true,
	Gripper = false,
	Lidar = false,
  Game = true,
}

--SJ: now we can have multiple FSM options
fsm.select = {
	Arm = 'RoboCup2015', 
	Body = 'RoboCup2015', 
	Gripper = 'DRCFinal',
	Head = 'RoboCup2015', 
	Motion = 'RoboCup2015'
}

-- Custom libraries
fsm.libraries = {
	MotionLib = 'DRCFinal',
	ArmLib = 'DRCFinal', --Now steve's libs are merged into one 
	World = 'RoboCup2015'
}

fsm.Game = {
  {'gamePreinit','init','gameInitial'}, --this is not initial (we manually init the robot)
  {'gameInitial', 'ready', 'gameReady'},
  {'gameInitial', 'set', 'gameSet'},
  {'gameReady', 'set', 'gameSet'},
  {'gameSet', 'play', 'gamePlaying'},
  {'gamePlaying', 'finish', 'gameStop'},

  {'gameInitial', 'finish', 'gameStop'},
  {'gameReady', 'finish', 'gameStop'},
  {'gameSet', 'finish', 'gameStop'},
  {'gamePlaying', 'finish', 'gameStop'},

  {'gameStop', 'init', 'gameInitial'},
}

fsm.Arm = {
  {'armDetect', 'timeout', 'armDetect'},  
}

fsm.Head = {
  {'headIdle', 'scan', 'headBackScan'},
  {'headIdle', 'teleop', 'headTeleop'},
  {'headIdle', 'scanobs', 'headObstacleScan'},


  {'headTeleop', 'log', 'headLog'},
  --

  {'headObstacleScan', 'done', 'headTrack'},
  {'headObstacleScan', 'backscan', 'headBackScanInit'},
  {'headObstacleScan', 'teleop', 'headTeleop'},


  {'headBackScanInit', 'ballfound', 'headTrack'},
  {'headBackScanInit', 'noball', 'headBackScanInit'},
  

  {'headScan', 'ballfound', 'headTrack'},
  {'headScan', 'noball', 'headBackScan'},
  {'headScan', 'teleop', 'headTeleop'},
  {'headScan', 'scanobs', 'headObstacleScan'},
  -- 
  {'headTeleop', 'scan', 'headBackScan'},
  {'headTeleop', 'scanobs', 'headObstacleScan'},
  {'headTeleop', 'scangoalie', 'headScan'},
  -- 
  
 

  {'headBackScan', 'ballfound', 'headTrack'},
  {'headBackScan', 'noball', 'headBackScan'},
  {'headBackScan', 'scan', 'headScan'},
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
  {'headKickFollow', 'teleop', 'headTrack'},

  {'headLookGoal', 'timeout', 'headTrack'},
  {'headLookGoal', 'scanobs', 'headObstacleScan'},  
  {'headLookGoal', 'teleop', 'headTeleop'},


  
  
}





--RC body FSM
--[[
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
--]]

--[[
fsm.Body = {
  {'bodyIdle', 'init', 'bodyInit'},
  {'bodyInit', 'done', 'bodyStop'},
	--
	{'bodyStop', 'approach', 'bodyApproachMessy'},
	{'bodyStop', 'stepflat', 'bodyApproachMessy'},

	{'bodyStop', 'stop', 'bodyStop'},
  --
  {'bodyApproachMessy', 'stop', 'bodyStop'},
	{'bodyApproachMessy', 'done', 'bodyStop'},

  {'bodyStop', 'stepover1', 'bodyStep'},
  {'bodyStep', 'done', 'bodyStop'},
  {'bodyStep', 'nextstep', 'bodyStep'},
}
--]]

fsm.Body = {
  {'bodyIdle', 'init', 'bodyInit'},
  {'bodyInit', 'done', 'bodyStop'},

--  {'bodyStop', 'footrace', 'bodyFootRace'},
--  {'bodyFootRace', 'done', 'bodyStop'},

  {'bodyStop', 'kick', 'bodyRobocupKick'},
--  {'bodyStop', 'play', 'bodyRobocupIdle'},
--  {'bodyStop', 'goalie', 'bodyRobocupGoalieIdle'},
  {'bodyStop', 'approach', 'bodyRobocupApproach'},





  {'bodyStop', 'play', 'bodyRobocupIdle'},
  {'bodyStop', 'goalie', 'bodyRobocupGoalieIdle'},

  {'bodyStop', 'uninit', 'bodyUninit'},
  {'bodyUninit', 'done', 'bodyIdle'},


	{'bodyRobocupIdle', 'timeout', 'bodyRobocupIdle'},
  {'bodyRobocupIdle', 'ballfound', 'bodyRobocupFollow'},
  {'bodyRobocupIdle','stop','bodyStop'},
  {'bodyRobocupIdle','goalie','bodyRobocupGoalieIdle'},

  {'bodyRobocupApproach', 'done', 'bodyRobocupKick'},
  {'bodyRobocupApproach', 'ballfar', 'bodyRobocupFollow'},
  {'bodyRobocupApproach','stop','bodyStop'},

  {'bodyRobocupFollow', 'done', 'bodyRobocupIdle'},
  {'bodyRobocupFollow', 'timeout', 'bodyRobocupFollow'},
  {'bodyRobocupFollow', 'ballclose', 'bodyRobocupApproach'},
  {'bodyRobocupFollow','stop','bodyStop'},

	{'bodyRobocupKick', 'done', 'bodyRobocupIdle'},
  {'bodyRobocupKick', 'testdone', 'bodyStop'},

  {'bodyRobocupGoalieIdle', 'attacker', 'bodyRobocupIdle'},
  {'bodyRobocupGoalieIdle','stop','bodyStop'},
}



fsm.Motion = {
	{'motionIdle', 'timeout', 'motionIdle'},
	{'motionIdle', 'stand', 'motionInit'},
	{'motionIdle', 'bias', 'motionBiasInit'},
	--
	{'motionBiasInit', 'done', 'motionBiasIdle'},
	{'motionBiasIdle', 'stand', 'motionInit'},
	--
	{'motionInit', 'done', 'motionStance'},
	{'motionUnInit', 'done', 'motionIdle'},
	--
	{'motionStance', 'bias', 'motionBiasInit'},
	{'motionStance', 'uninit', 'motionUnInit'},
	{'motionStance', 'hybridwalk', 'motionHybridWalkInit'},
  {'motionStance', 'done_step', 'motionHybridWalkKick'},
--	{'motionStance', 'slowstep', 'motionSlowStep'},

	--
	{'motionHybridWalkInit', 'done', 'motionHybridWalk'},
	{'motionHybridWalk', 'done', 'motionHybridWalkEnd'},
	{'motionHybridWalk', 'emergency', 'motionStance'},
	{'motionHybridWalkEnd', 'done', 'motionStance'},

	--kick handlings
  {'motionHybridWalk', 'done_step', 'motionHybridWalkKick'},
  {'motionHybridWalkKick', 'done', 'motionStance'},
  {'motionHybridWalkKick', 'walkalong', 'motionHybridWalk'},

	--
	{'motionStepPreview', 'done', 'motionStance'},
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
  tLost = 10,
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


  rCircle = 0.6,
  

}

fsm.bodyRobocupApproach = {
  target={0.30,0.12} ,
  th = {0.34, 0.02}
}


Config.fsm = fsm

return Config
