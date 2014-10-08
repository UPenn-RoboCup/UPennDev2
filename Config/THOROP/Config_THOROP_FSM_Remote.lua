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
  {'motionStance', 'remote', 'motionRemote'},
  --
  {'motionRemote', 'quit', 'motionStance'},
  {'motionRemote', 'timeout', 'motionRemote'},
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

return Config
