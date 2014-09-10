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
  'Body',
  'Motion',
}

--SJ: now we can have multiple FSM options 
fsm.select = {
  Body = 'Teach',
  Motion = 'Teach'
}


fsm.Body = {
  {'bodyIdle', 'init', 'bodyInit'},
  {'bodyInit', 'done', 'bodyStop'},

  {'bodyStop', 'footrace', 'bodyFootRace'},
  {'bodyFootRace', 'done', 'bodyStop'},

  {'bodyStop', 'stepinplace', 'bodyStepPlace'},
--  {'bodyStop', 'stepwaypoint', 'bodyStepWaypoint'},
  {'bodyStop', 'kick', 'bodyRobocupKick'},
  {'bodyStop', 'play', 'bodyRobocupIdle'},
  {'bodyStop', 'goalie', 'bodyRobocupGoalieIdle'},

  {'bodyStop', 'approach', 'bodyRobocupApproach'},

}


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
