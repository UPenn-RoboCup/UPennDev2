assert(Config, 'Need a pre-existing Config table!')

Config.fsm = {}

-- Which FSMs should be enabled?
Config.fsm.enabled = {
  'Body', 'Arm'
}

Config.fsm.Arm = {
  {'armIdle', 'timeout', 'armIdle'},
  --{'armIdle', 'init', 'armInit'},
  --
  --{'armInit', 'teleop', 'armTeleop'},
}

Config.fsm.Body = {
  {'bodyIdle', 'timeout', 'bodyIdle'},
  {'bodyIdle', 'init', 'bodyInit'},
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
