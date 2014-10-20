assert(Config, 'Need a pre-existing Config table!')

local fsm = {}

Config.demo = false
--Config.demo = true

Config.torque_legs = true

-- Update rate in Hz
fsm.update_rate = 100

-- Which FSMs should be enabled?
fsm.enabled = {
--  'Body',
	'Arm',
  'Motion',
}

--SJ: now we can have multiple FSM options 
fsm.select = {
	Arm = 'Teach',
  Body = 'Teach',
  Motion = 'Teach'
}

fsm.Body = {
  {'bodyIdle', 'init', 'bodyInit'},
  --
  {'bodyInit', 'done', 'bodyStop'},
}

fsm.Arm = {
	-- Idle
  {'armIdle', 'timeout', 'armIdle'},
	{'armIdle', 'init', 'armInit'},
	-- Init
  {'armInit', 'done', 'armStance'},
	-- Stance
	{'armStance', 'timeout', 'armStance'},
	{'armStance', 'ready', 'armReady'},
	-- Ready
	{'armReady', 'timeout', 'armReady'},
}

fsm.Motion = {
  -- Idle
  {'motionIdle', 'timeout', 'motionIdle'},
  {'motionIdle', 'stand', 'motionInit'},
  -- Init
  {'motionInit', 'done', 'motionStance'},
  {'motionInit', 'timeout', 'motionInit'},
  --
  {'motionStance', 'sway', 'motionSway'},
  {'motionStance', 'lean', 'motionLean'},
  --
  {'motionSway', 'lean', 'motionLean'},
  {'motionSway', 'switch', 'motionSway'},
  {'motionSway', 'timeout', 'motionSway'},
  {'motionSway', 'stand', 'motionStance'},
  --
  {'motionLean', 'stepup', 'motionLift'},
  {'motionLean', 'stepdown', 'motionStepDown'},
  {'motionLean', 'stand', 'motionInit'},
  --
  {'motionLift', 'lean', 'motionLean'},
  {'motionLift', 'timeout', 'motionLower'},
  {'motionLift', 'quit', 'motionLower'},
  --{'motionLift', 'done', 'motionLower'},
  {'motionLift', 'done', 'motionHold'},
  --
  {'motionHold', 'done', 'motionLower'},
  --
  {'motionLower', 'flat', 'motionStance'},
  {'motionLower', 'uneven', 'motionCaptainMorgan'},
  --
  {'motionCaptainMorgan', 'stepup', 'motionStepUp'},
  {'motionCaptainMorgan', 'stepdown', 'motionJoin'},
  --
  {'motionStepUp', 'done', 'motionHold'},
  --
  {'motionStepDown', 'done', 'motionLower'},
  --
  {'motionJoin', 'done', 'motionLower'},
}

Config.fsm = fsm

return Config
