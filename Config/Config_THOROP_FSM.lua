assert(Config, 'Need a pre-existing Config table!')

local fsm = {}

-- Do we disable FSMs?
fsm.disabled = false
-- fsm.disabled = true

-- Do we disable Kick?
--fsm.disable_kick = true
fsm.disable_kick = false

-- Update rate in Hz
fsm.update_rate = 100

-- Which FSMs should be enabled?
fsm.enabled = {
--  'Arm',
  'Body',
  'Head',
  'Motion',
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

  {'bodyStop', 'stepinplace', 'bodyStepPlace'},
--  {'bodyStop', 'stepwaypoint', 'bodyStepWaypoint'},
  {'bodyStop', 'kick', 'bodyRobocupKick'},
  {'bodyStop', 'play', 'bodyRobocupIdle'},
  {'bodyStop', 'goalie', 'bodyRobocupGoalieIdle'},

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

assert(Config.dev.walk, 'Need a walk engine specification')
fsm.Motion = {
  {'motionIdle', 'timeout', 'motionIdle'},
  {'motionIdle', 'stand', 'motionInit'},
  {'motionIdle', 'bias', 'motionBiasInit'},

  {'motionBiasInit', 'done', 'motionBiasIdle'}, 
  {'motionBiasIdle', 'stand', 'motionInit'}, 

  {'motionInit', 'done', 'motionStance'},

  {'motionStance', 'bias', 'motionBiasInit'},
  {'motionStance', 'preview', 'motionStepPreview'},
  {'motionStance', 'walk', Config.dev.walk},
  {'motionStance', 'kick', 'motionKick'},
  {'motionStance', 'done_step', 'motionHybridWalkKick'},

  {'motionStance', 'sit', 'motionSit'},
  {'motionSit', 'stand', 'motionStandup'},
  {'motionStandup', 'done', 'motionStance'},

  {'motionStepPreview', 'done', 'motionStance'},
  {Config.dev.walk, 'done', 'motionStance'},
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

Config.fsm = fsm

-- Add all FSM directories that are in Player
for _,sm in ipairs(Config.fsm.enabled) do
  local pname = {HOME, '/Player/', sm, 'FSM', '/?.lua;', package.path}
  package.path = table.concat(pname)
end

return Config
