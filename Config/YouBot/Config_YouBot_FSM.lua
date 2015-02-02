assert(Config, 'Need a pre-existing Config table!')
local vector = require'vector'

local fsm = {}
-- Update rate in Hz
fsm.update_rate = 100

-- Which FSMs should be enabled?
fsm.enabled = {
  'Arm',
}

-- Timeouts are generally not needed, but in planning places, it is
-- Less clutter if not transitions here. But always program them in the state
-- For instance, lineIter from libPlan should *always* have a timeout
fsm.Arm = {
  --{'armIdle', 'timeout', 'armIdle'},
  {'armIdle', 'init', 'armInit'},
  --
  {'armInit', 'timeout', 'armInit'},
  {'armInit', 'done', 'armStance'},
  {'armInit', 'init', 'armInit'},
  --
  {'armStance', 'timeout', 'armStance'},
  {'armStance', 'wire', 'armWireLook'},
  {'armStance', 'init', 'armInit'},
  --{'armStance', 'teleop', 'armTeleop'},
  --
  {'armWireLook', 'timeout', 'armWireLook'},
  {'armWireLook', 'lost', 'armInit'}, -- Initial pose if lost the wire
  {'armWireLook', 'done', 'armWireApproach'}, -- Initial pose if lost the wire
  {'armWireLook', 'init', 'armInit'},
  --
  {'armWireApproach', 'timeout', 'armWireApproach'},
  {'armWireApproach', 'lost', 'armInit'},
  {'armWireApproach', 'far', 'armWireLook'}, -- If not well aligned, realign
  {'armWireApproach', 'close', 'armWireGrip'}, -- If not well aligned, realign
  {'armWireApproach', 'init', 'armInit'},
  --
  --{'armWireGrip', 'timeout', 'armInit'},
  {'armWireGrip', 'timeout', 'armIdle'},
  {'armWireGrip', 'init', 'armInit'},
}

-- State specific tuning params
fsm.armInit = {
  qLArm = vector.new{0, -55, 110, 55, 0} * DEG_TO_RAD
}
-- This state machine has stance and init with same goal
fsm.armStance = {
  qLArm = fsm.armInit.qLArm
}
-- Constantly look at the wire, lining it up to be centered and vertical
fsm.armWireLook = {
  lost_timeout = 2.0,
  thresh_yaw = 3 * DEG_TO_RAD,
  thresh_roll = 3 * DEG_TO_RAD,
  roll_rate = 0.500,
  yaw_rate = 0.500,
}
--
fsm.armWireApproach = {
  lost_timeout = 1.0,
  thresh_yaw = 5 * DEG_TO_RAD,
  thresh_roll = 6 * DEG_TO_RAD,
  roll_rate = 0.200,
  yaw_rate = 0.300,
  approach_rate = 0.0001,
  wire_close = .16--0.075, --closest
}

fsm.Body = {
  {'bodyIdle', 'init', 'bodyInit'},
  {'bodyIdle', 'timeout', 'bodyIdle'},
  --
  {'bodyInit', 'path', 'bodyPath'},
  {'bodyInit', 'grab', 'bodyApproachGrab'},
  --
  {'bodyPath', 'timeout', 'bodyPath'}, -- replan on timeout
  {'bodyPath', 'path', 'bodyPath'}, -- Call replan, essentially
  {'bodyPath', 'done', 'bodyApproachGrab'}, -- TODO: Call the ArmFSM
  --
  {'bodyApproachGrab', 'done', 'bodyGrab'}, -- TODO: Call the ArmFSM
  {'bodyApproachGrab', 'far', 'bodyPath'},
  --
  {'bodyGrab', 'done', 'bodyInit'}, -- Go to the initial pose
}

Config.fsm = fsm

-- Add all FSM directories that are in Player
for _,sm in ipairs(fsm.enabled) do
  local pname = {HOME, '/Player/', sm, 'FSM', '/?.lua;', package.path}
  package.path = table.concat(pname)
end

return Config
