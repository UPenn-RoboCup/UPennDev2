assert(Config, 'Need a pre-existing Config table!')
local vector = require'vector'

Config.fsm = {}
-- Update rate in Hz
Config.fsm.update_rate = 100

-- Which FSMs should be enabled?
Config.fsm.enabled = {
  'Body', 'Arm',
}

Config.fsm.Arm = {
  {'armIdle', 'timeout', 'armIdle'},
  {'armIdle', 'init', 'armInit'},
  --
  {'armInit', 'timeout', 'armInit'},
  --{'armInit', 'teleop', 'armTeleop'},
}

-- State specific tuning params
Config.fsm.armInit = {
  qLArm = vector.zeros(Config.nJoint)
}

Config.fsm.Body = {
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

-- Add all FSM directories that are in Player
for _,sm in ipairs(Config.fsm.enabled) do
  local pname = {HOME, '/Player/', sm, 'FSM', '/?.lua;', package.path}
  package.path = table.concat(pname)
end

return Config
