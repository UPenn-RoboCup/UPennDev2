assert(Config, 'Need a pre-existing Config table!')

-- Override! Disable the wizards
-- Based on the master config, our fsm config has the highest priority!
Config.wizards = {}
Config.sensors = {ft = true}

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
  Body = 'Teach',
  Motion = 'Teach'
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
  {'motionStance', 'lean', 'motionLean'},
  {'motionStance', 'sway', 'motionSway'},
  --
  {'motionSway', 'lean', 'motionLean'},
  {'motionSway', 'switch', 'motionSway'},
  {'motionSway', 'timeout', 'motionStance'},
  {'motionSway', 'stand', 'motionStance'},
  --
  {'motionLean', 'sway', 'motionSway'},
  {'motionLean', 'done', 'motionLift'},
  {'motionLean', 'stand', 'motionInit'},
  --
  --{'motionLift', 'lean', 'motionLean'},
  {'motionLift', 'timeout', 'motionLower'},
  {'motionLift', 'quit', 'motionLower'},
  --{'motionLift', 'done', 'motionLower'},
  {'motionLift', 'done', 'motionHold'},
  --
  {'motionHold', 'done', 'motionLower'},
  --
  {'motionLower', 'flat', 'motionStance'},
  {'motionLower', 'uneven', 'motionStance'},
}

Config.fsm = fsm

for _,sm in ipairs(Config.fsm.enabled) do
  if Config.fsm.select[sm] then
    local pname = {HOME, '/Player/', sm, 'FSM/',Config.fsm.select[sm], '/?.lua;', package.path}
    package.path = table.concat(pname)
  else --default fsm
    local pname = {HOME, '/Player/', sm, 'FSM', '/?.lua;', package.path}
    package.path = table.concat(pname)
  end  
end

Config.stop_at_neutral = true --false for walk testing

Config.demo = false
--Config.demo = true

--  Config.use_walkkick = true
Config.use_walkkick = false

--Config.torque_legs = false
Config.torque_legs = true

return Config
