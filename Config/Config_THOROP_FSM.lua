assert(Config, 'Need a pre-existing Config table!')

Config.fsm = {}

-- Do we disable FSMs?
Config.fsm.disabled = false

-- Update rate in Hz
Config.fsm.update_rate = 100

-- Which FSMs should be enabled?
Config.fsm.enabled = {
--'Arm',
--'Body',
--'Lidar',
'Motion'
}

Config.fsm.Arm = {
  {'armIdle', 'timeout', 'armIdle'},
  --
  {'armIdle', 'init', 'armInit'},
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
assert(Config.dev.walk, 'Need a walk engine specification')
Config.fsm.Motion = {
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

-- Add all FSM directories that are in Player
for _,sm in ipairs(Config.fsm.enabled) do
  local pname = {HOME, '/Player/', sm, 'FSM', '/?.lua;', package.path}
  package.path = table.concat(pname)
end

return Config
