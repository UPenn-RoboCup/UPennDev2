assert(Config, 'Need a pre-existing Config table!')

local fsm = {}

-- Update rate in Hz
fsm.update_rate = 100

-- Which FSMs should be enabled?
fsm.enabled = {
--  'Body',
  'Motion',
}

--SJ: now we can have multiple FSM options 
fsm.select = {
--  Body = 'Remote',
  Motion = 'Remote'
}

fsm.Body = {
  {'bodyIdle', 'init', 'bodyInit'},
  --
  {'bodyInit', 'done', 'bodyStop'},
}

fsm.Motion = {
  -- Idle
  {'motionIdle', 'timeout', 'motionIdle'},
  {'motionIdle', 'stand', 'motionInit'},
  -- Init
  {'motionInit', 'done', 'motionStance'},
  {'motionInit', 'timeout', 'motionInit'},
  --
  {'motionStance', 'remote', 'motionIntermediate'},
  {'motionIntermediate', 'done', 'motionRemote'},
  {'motionIntermediate', 'timeout', 'motionIntermediate'},
  --
  --{'motionRemote', 'quit', 'motionStance'},
  {'motionRemote', 'timeout', 'motionRemote'},
}

Config.fsm = fsm

return Config
