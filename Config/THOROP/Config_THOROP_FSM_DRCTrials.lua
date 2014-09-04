assert(Config, 'Need a pre-existing Config table!')

local fsm = {}

-- Do we disable FSMs?
fsm.disabled = false

-- Do we disable Kick?
--fsm.disable_kick = true
fsm.disable_kick = false

-- Update rate in Hz
fsm.update_rate = 100

-- Which FSMs should be enabled?
fsm.enabled = {
  'Arm',
  'Body',
  'Head',
  'Motion',
  'Lidar'
}

--SJ: now we can have multiple FSM options 
fsm.select = {
  Arm = 'DRCTrials',  
  Head = 'Default',
  Body = 'Default',
}


fsm.Lidar = {
  {'lidarIdle', 'pan', 'lidarPan'},
  {'lidarPan', 'switch', 'lidarPan'},
  {'lidarPan', 'stop', 'lidarIdle'},
}

fsm.Head = {  
  {'headIdle', 'teleop', 'headTeleop'},
}

fsm.Body = {
  {'bodyIdle', 'init', 'bodyInit'},
  {'bodyInit', 'done', 'bodyStop'},

  {'bodyStop', 'stepinplace', 'bodyStepPlace'},
  {'bodyStepPlace',   'done', 'bodyStop'},
--  {'bodyStepWaypoint',   'done', 'bodyStop'},
}

fsm.Arm = {
  {'armIdle', 'timeout', 'armIdle'},
  {'armIdle', 'drive', 'armDrive'},
  {'armIdle', 'init', 'armInit'},

  {'armInit', 'done', 'armPose1'},

  
  {'armPose1', 'teleop', 'armTeleop'},
  --{'armPose1', 'teleop', 'armIKTest'},
  --{'armIKTest', 'teleop', 'armPose1'},

  {'armPose1', 'pushdoorgrab', 'armPushDoorSideGrip'},
  {'armPose1', 'doorgrab', 'armPullDoorSideGrip'},

  {'armPose1', 'toolgrab', 'armToolGrip'},
  {'armPose1', 'hosegrab', 'armHoseGrip'},


  {'armPose1', 'debrisgrab', 'armDebrisGrip'},
  {'armPose1', 'smallvalvegrab', 'armSmallValveGrip'},
  {'armPose1', 'barvalvegrab', 'armBarValveGrip'},
  {'armPose1', 'smallvalverightgrab', 'armSmallValveRightGrip'},
  {'armPose1', 'barvalverightgrab', 'armBarValveRightGrip'},


  {'armSmallValveGrip', 'done', 'armPose1'},
  {'armSmallValveRightGrip', 'done', 'armPose1'},
  {'armBarValveGrip', 'done', 'armPose1'},
  {'armBarValveRightGrip', 'done', 'armPose1'},

  {'armToolGrip', 'done', 'armPose1'},
  {'armToolGrip', 'hold', 'armToolHold'},
  {'armToolHold', 'toolgrab', 'armToolChop'},
  {'armToolChop', 'done', 'armToolHold'},

  {'armHoseGrip', 'done', 'armPose1'},
  {'armHoseGrip', 'hold', 'armHoseHold'},
  {'armHoseHold', 'hold', 'armHoseHold'},
  {'armHoseHold', 'hosegrab', 'armHoseTap'},
  {'armHoseTap', 'done', 'armHoseHold'},

  {'armPushDoorSideGrip', 'done', 'armPose1'},
  {'armPullDoorSideGrip', 'done', 'armPose1'},

  {'armDebrisGrip', 'done', 'armPose1'},
  {'armTeleop', 'done', 'armPose1'},
}


assert(Config.dev.walk, 'Need a walk engine specification')
fsm.Motion = {
  {'motionIdle', 'timeout', 'motionIdle'},
  {'motionIdle', 'stand', 'motionInit'},
  {'motionIdle', 'bias', 'motionBiasInit'},

  {'motionBiasInit', 'done', 'motionBiasIdle'}, 
  {'motionBiasIdle', 'stand', 'motionInit'}, 

  {'motionInit', 'done', 'motionStance'},

  {'motionStance', 'bias', 'motionBiasInit'},
  {'motionStance', 'preview', 'motionStepPreview'},
  {'motionStance', 'walk', Config.dev.walk},
  {'motionStance', 'kick', 'motionKick'},
  {'motionStance', 'done_step', 'motionHybridWalkKick'},

  {'motionStance', 'sit', 'motionSit'},
  {'motionSit', 'stand', 'motionStandup'},
  {'motionStandup', 'done', 'motionStance'},

  {'motionStepPreview', 'done', 'motionStance'},
  {Config.dev.walk, 'done', 'motionStance'},
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

fsm.dqNeckLimit = {
  60 * DEG_TO_RAD, 60 * DEG_TO_RAD
}

--HeadReady
fsm.headReady = {
  dist = 3
}



if IS_WEBOTS then
end

Config.fsm = fsm

--[[
-- Add all FSM directories that are in Player
for _,sm in ipairs(Config.fsm.enabled) do
  local pname = {HOME, '/Player/', sm, 'FSM', '/?.lua;', package.path}
  package.path = table.concat(pname)
end
--]]

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
