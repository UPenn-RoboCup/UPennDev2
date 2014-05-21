assert(Config, 'Need a pre-existing Config table!')

Config.fsm = {}
-- Update rate in Hz
Config.fsm.update_rate = 100
Config.fsm.enabled = false

-- Which FSMs should be enabled?
Config.fsm.enabled = {'Arm', 'Body', 'Lidar', 'Motion'}

Config.fsm.Arm = {
  {'armIdle', 'timeout', 'armIdle'},
  --{'armIdle', 'init', 'armInit'},
  --
  --{'armInit', 'teleop', 'armTeleop'},
}

Config.fsm.Body = {
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

-- Add all FSM directories that are in Player
for _,sm in ipairs(Config.fsm.enabled) do
  local pname = {HOME, '/Player/', sm, 'FSM', '/?.lua;', package.path}
  package.path = table.concat(pname)
end

return Config
