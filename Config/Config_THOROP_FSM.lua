assert(Config, 'Need a pre-existing Config table!')

local fsm = {}

-- Do we disable FSMs?
fsm.disabled = false

-- Update rate in Hz
fsm.update_rate = 100

-- Which FSMs should be enabled?
fsm.enabled = {
--'Arm',
--'Body',
--'Lidar',
'Motion'
}

fsm.Arm = {
  {'armIdle', 'timeout', 'armIdle'},
  --
  {'armIdle', 'init', 'armInit'},
  --
  --{'armInit', 'teleop', 'armTeleop'},
}

fsm.Body = {
  {'bodyIdle', 'init', 'bodyInit'},
  {'bodyIdle', 'drive', 'bodyDrive'},
  {'bodyIdle', 'follow', 'bodyStepWaypoint'},
  {'bodyIdle', 'stepover', 'bodyStepOver'},
  {'bodyIdle', 'stepplan2', 'bodyStepPlan2'},
  --
  {'bodyInit', 'done', 'bodyIdle'},
  {'bodyStepWaypoint', 'done', 'bodyIdle'},
  {'bodyStepOver', 'done', 'bodyIdle'},
  {'bodyStepWiden', 'done', 'bodyIdle'},
  {'bodyStepPlan2', 'done', 'bodyIdle'},
}
assert(Config.dev.walk, 'Need a walk engine specification')
fsm.Motion = {
  {'motionIdle', 'timeout', 'motionIdle'},
  {'motionIdle', 'stand', 'motionInit'},
  {'motionIdle', 'drive', 'motionDrive'},
  {'motionIdle', 'bias', 'motionBiasInit'},
  --
  {'motionInit', 'done', 'motionStance'},
  --
  {'motionIdle', 'bias', 'motionBiasInit'},
  --
  {'motionBiasInit', 'bias', 'motionInit'},
  --
  {'motionStance', 'bias', 'motionBiasInit'},
  {'motionStance', 'preview', 'motionStepPreview'},
  {'motionStance', 'walk', Config.dev.walk},
  --
  {'motionStepPreview', 'done', 'motionStance'},
  --
  {Config.dev.walk, 'done', 'motionStance'},
}

fsm.dqNeckLimit = {90*DEG_TO_RAD,90*DEG_TO_RAD}

fsm.headScan = {
  pitch0 = 30*DEG_TO_RAD,
  pitchMag = 20*DEG_TO_RAD,
  yawMag = 60*DEG_TO_RAD,
  tScan = 8, --sec
}

--HeadReady
fsm.headReady = {
  dist = 3
}

--HeadTrack
fsm.headTrack = {}
fsm.headTrack.tLost = 2
fsm.headTrack.timeout = 3

--HeadLookGoal: Look up to see the goal
fsm.headLookGoal = {
  yawSweep = 50*DEG_TO_RAD,
  tScan = 1.0,
  minDist = 0.40,
}

--HeadLookGoal: Look up to see the goal
fsm.headLookGoal = {}
fsm.headLookGoal.yawSweep = 50*DEG_TO_RAD
fsm.headLookGoal.tScan = 1.0
fsm.headLookGoal.minDist = 0.40

--HeadSweep: Look around to find the goal
fsm.headSweep = {}
fsm.headSweep.tScan = 1.0
fsm.headSweep.tWait = 0.25

Config.fsm = fsm

-- Add all FSM directories that are in Player
for _,sm in ipairs(Config.fsm.enabled) do
  local pname = {HOME, '/Player/', sm, 'FSM', '/?.lua;', package.path}
  package.path = table.concat(pname)
end

return Config
