assert(Config, 'Need a pre-existing Config table!')

local fsm = {}

-- Do we disable FSMs?
fsm.disabled = false
-- fsm.disabled = true

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
--  {'headIdle', 'scan', 'headScan'},

  {'headIdle', 'scan', 'headBackScan'},
  {'headIdle', 'teleop', 'headTeleop'},

  {'headBackScan', 'ballfound', 'headTrack'},
  {'headBackScan', 'noball', 'headBackScan'},
  {'headBackScan', 'teleop', 'headTeleop'},

  {'headScan', 'ballfound', 'headTrack'},
  {'headScan', 'noball', 'headBackScan'},
  {'headScan', 'teleop', 'headTeleop'},
  --
  {'headTrack', 'balllost', 'headScan'},
  {'headTrack', 'timeout', 'headLookGoal'},
  {'headTrack', 'sweep', 'headSweep'},
  {'headTrack', 'teleop', 'headTeleop'},
  --
  {'headLookGoal', 'timeout', 'headTrack'},
  --{'headLookGoal', 'lost', 'headSweep'},
  --
  {'headSweep', 'done', 'headTrack'},
  --
  {'headTeleop', 'scan', 'headBackScan'},
}

fsm.Body = {
  {'bodyIdle', 'init', 'bodyInit'},
  {'bodyInit', 'done', 'bodyStop'},

  {'bodyStop', 'stepinplace', 'bodyStepPlace'},
  {'bodyStop', 'stepwaypoint', 'bodyStepWaypoint'},
  {'bodyStop', 'play', 'bodyRobocupIdle'},
  {'bodyStop', 'kick', 'bodyRobocupKick'},

  {'bodyStepPlace',   'done', 'bodyStop'},
  {'bodyStepWaypoint',   'done', 'bodyStop'},
  
  {'bodyRobocupIdle', 'timeout', 'bodyRobocupIdle'},
  {'bodyRobocupIdle', 'ballfound', 'bodyRobocupFollow'},
  {'bodyRobocupIdle','stop','bodyStop'},

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

}

assert(Config.dev.walk, 'Need a walk engine specification')
fsm.Motion = {
  {'motionIdle', 'timeout', 'motionIdle'},
  {'motionIdle', 'stand', 'motionInit'},
  {'motionIdle', 'bias', 'motionBiasInit'},

  {'motionBiasInit', 'bias', 'motionInit'},  
  {'motionInit', 'done', 'motionStance'},

  {'motionStance', 'bias', 'motionBiasInit'},
  {'motionStance', 'preview', 'motionStepPreview'},
  {'motionStance', 'walk', Config.dev.walk},
  {'motionStance', 'kick', 'motionKick'},

  {'motionStepPreview', 'done', 'motionStance'},
  {Config.dev.walk, 'done', 'motionStance'},
  {'motionKick', 'done', 'motionStance'},

--For new hybrid walk
  {'motionStance', 'hybridwalk', 'motionHybridWalkInit'},
  {'motionHybridWalkInit', 'done', 'motionHybridWalk'},

  {'motionHybridWalk', 'done', 'motionStance'},
  {'motionHybridWalk', 'done', 'motionHybridWalkEnd'},
  
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
  tLost = 2,
  timeout = 10,
}

--HeadLookGoal: Look up to see the goal
fsm.headLookGoal = {
  yawSweep = 50*DEG_TO_RAD,
  tScan = 1.0,
  minDist = 0.40,
}

--HeadSweep: Look around to find the goal
fsm.headSweep = {
  tScan = 1.0,
  tWait = 0.25,
}

fsm.bodyRobocupFollow = {
  th_lfoot = 0.001,
  th_rfoot = 0.001,
  th_dist = 0.08,  --TODO
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
