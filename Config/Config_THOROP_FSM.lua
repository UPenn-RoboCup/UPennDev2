assert(Config, 'Need a pre-existing Config table!')

local fsm = {}

-- Do we disable FSMs?
fsm.disabled = false
-- fsm.disabled = true

-- Update rate in Hz
fsm.update_rate = 100

-- Which FSMs should be enabled?
fsm.enabled = {
  'Arm',
  --'Lidar',
  'Body',
  'Head',
  'Motion',
}

fsm.Arm = {
  {'armIdle', 'timeout', 'armIdle'},
  {'armIdle', 'init', 'armInit'},
  --
  {'armInit', 'done', 'armPose1'},

}

local vector = require'vector'
fsm.armInit = {
  qLArm = vector.new({118.96025904076,9.0742631178663,-5,-81.120944928286,81,14.999999999986, 9})*DEG_TO_RAD
}

fsm.Head = {
--  {'headIdle', 'teleop', 'headTeleop'},
--  {'headIdle', 'center', 'headCenter'},
  {'headIdle', 'scan', 'headScan'},
  --
--  {'headTeleop', 'reset', 'headIdle'},
--  {'headTeleop', 'center', 'headCenter'},
  --
--  {'headCenter', 'done', 'headIdle'},
  --
  {'headScan', 'ballfound', 'headTrack'},
  {'headScan', 'center', 'headCenter'},
  --
  {'headTrack', 'balllost', 'headScan'},
--  {'headTrack', 'center', 'headCenter'},
  --{'headTrack', 'timeout', 'headLookGoal'},
  {'headTrack', 'sweep', 'headSweep'},
  --
  --{'headLookGoal', 'timeout', 'headTrack'},
  --{'headLookGoal', 'lost', 'headSweep'},
  {'headSweep', 'done', 'headTrack'},
}

--[[
-- For DRC
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
--]]

-- For RoboCup
fsm.Body = {
  {'bodyIdle', 'init', 'bodyInit'},
  {'bodyInit', 'done', 'bodyRobocupIdle'},
  --
  {'bodyRobocupIdle', 'ballfound', 'bodyRobocupFollow'},
  {'bodyRobocupIdle', 'follow', 'bodyStepWaypoint'},
  {'bodyRobocupIdle', 'stepplan2', 'bodyStepPlan2'},
  --
  {'bodyRobocupFollow', 'done', 'bodyRobocupIdle'},
  --
  {'bodyStepWaypoint',   'done', 'bodyRobocupIdle'},
  --
  {'bodyStepPlan2',   'done', 'bodyRobocupIdle'}
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

fsm.dqNeckLimit = {
  90 * DEG_TO_RAD, 90 * DEG_TO_RAD
}

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
fsm.headTrack = {
  tLost = 2,
  timeout = 3,
}

--HeadLookGoal: Look up to see the goal
fsm.headLookGoal = {
  yawSweep = 50*DEG_TO_RAD,
  tScan = 1.0,
  minDist = 0.40,
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

Config.fsm = fsm

-- Add all FSM directories that are in Player
for _,sm in ipairs(Config.fsm.enabled) do
  local pname = {HOME, '/Player/', sm, 'FSM', '/?.lua;', package.path}
  package.path = table.concat(pname)
end

return Config
