assert(Config, 'Need a pre-existing Config table!')

local fsm = {}

Config.demo = false
--Config.demo = true

Config.torque_legs = true

-- Update rate in Hz
fsm.update_rate = 100

-- Which FSMs should be enabled?
fsm.enabled = {
  'Body',
	'Arm',
  'Motion',
	'Head'
}

--SJ: now we can have multiple FSM options 
fsm.select = {
	Arm = 'Teach',
  Body = 'Teach',
	Head = 'Teach',
  Motion = 'Teach'
}

fsm.Body = {
  {'bodyIdle', 'init', 'bodyInit'},
  --
  {'bodyInit', 'done', 'bodyStop'},
}

fsm.Head = {
  {'headIdle', 'init', 'headCenter'},
	{'headCenter', 'trackhand', 'headTrackHand'},
}

fsm.Arm = {
	-- Idle
  {'armIdle', 'timeout', 'armIdle'},
	{'armIdle', 'init', 'armInit'},
	-- Init
	{'armInit', 'timeout', 'armInit'},
  {'armInit', 'done', 'armStance'},
	-- Stance pose (for walking)
	{'armStance', 'timeout', 'armStance'},
	{'armStance', 'ready', 'armReady'},
	{'armStance', 'teleop', 'armTeleop'},
	-- Ready pose (for manipulating)
	{'armReady', 'timeout', 'armReady'},
	{'armReady', 'done', 'armTeleop'},
	{'armReady', 'teleop', 'armTeleop'},
	-- Teleop
	{'armTeleop', 'timeout', 'armTeleop'},
	{'armTeleop', 'init', 'armInit'},
	{'armTeleop', 'ready', 'armReady'},
	{'armTeleop', 'poke', 'armPoke'},
	-- Poke
	{'armPoke', 'timeout', 'armPoke'},
	{'armPoke', 'done', 'armTeleop'},
	{'armPoke', 'touch', 'armTeleop'},
}

--[[
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
--]]

fsm.select.Motion = 'RoboCup'
fsm.Motion = {
  {'motionIdle', 'timeout', 'motionIdle'},
  {'motionIdle', 'stand', 'motionInit'},
  {'motionIdle', 'bias', 'motionBiasInit'},

  {'motionBiasInit', 'done', 'motionBiasIdle'}, 
  {'motionBiasIdle', 'stand', 'motionInit'}, 

  {'motionInit', 'done', 'motionStance'},

  {'motionStance', 'bias', 'motionBiasInit'},
  {'motionStance', 'preview', 'motionStepPreview'},
  {'motionStance', 'kick', 'motionKick'},
  {'motionStance', 'done_step', 'motionHybridWalkKick'},

  {'motionStance', 'sit', 'motionSit'},
  {'motionSit', 'stand', 'motionStandup'},
  {'motionStandup', 'done', 'motionStance'},

  {'motionStepPreview', 'done', 'motionStance'},
  {'motionKick', 'done', 'motionStance'},

--For new hybrid walk
  {'motionStance', 'hybridwalk', 'motionHybridWalkInit'},
  {'motionHybridWalkInit', 'done', 'motionHybridWalk'},

  {'motionHybridWalk', 'done', 'motionStance'},
  {'motionHybridWalk', 'done', 'motionHybridWalkEnd'},

  {'motionHybridWalk', 'done_step', 'motionHybridWalkKick'},
  {'motionHybridWalkKick', 'done', 'motionStance'},
  {'motionHybridWalkKick', 'walkalong', 'motionHybridWalk'},
  
--  {'motionHybridWalk', 'done_step', 'motionStepNonstop'},
--  {'motionStepNonstop', 'done', 'motionStance'},

  {'motionHybridWalkEnd', 'done', 'motionStance'},

}

Config.fsm = fsm

return Config
