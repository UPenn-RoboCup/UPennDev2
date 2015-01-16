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
--  'Lidar'
}

--SJ: now we can have multiple FSM options 
fsm.select = {
  Arm = 'DRCFinal',  
  Head = 'DRCFinal',
  Body = 'DRCFinal',
  Motion = 'DRCFinal'
}

--SJ: custom library selection moved to HERE
fsm.libraries = {
  ArmLib = 'DRCFinal',
  MotionLib = 'DRCFinal',  
  World = 'DRCFinal'
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
  
  {'bodyStop', 'stepover1', 'bodyStep'},
  {'bodyStep', 'done', 'bodyStop'},
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



fsm.Motion = {
  {'motionIdle', 'timeout', 'motionIdle'},
  {'motionIdle', 'stand', 'motionInit'},
  {'motionInit', 'done', 'motionStance'},

  {'motionIdle', 'bias', 'motionBiasInit'},  
  {'motionStance', 'bias', 'motionBiasInit'},
  {'motionBiasInit', 'done', 'motionBiasIdle'}, 
  {'motionBiasIdle', 'stand', 'motionInit'}, 


  {'motionStance', 'preview', 'motionStepPreview'},
  {'motionStepPreview', 'done', 'motionStance'},


  {'motionStance', 'stair', 'motionStepPreviewStair'},
  {'motionStepPreviewStair', 'done', 'motionStepPreviewStairStopped'},
  {'motionStepPreviewStairStopped', 'stair', 'motionStepPreviewStair'},


  {'motionStance', 'hybridwalk', 'motionHybridWalkInit'},
  {'motionHybridWalkInit', 'done', 'motionHybridWalk'},
  {'motionHybridWalk', 'done', 'motionHybridWalkEnd'},
  {'motionHybridWalkEnd', 'done', 'motionStance'},
}

fsm.dqNeckLimit = {
  60 * DEG_TO_RAD, 60 * DEG_TO_RAD
}

Config.fsm = fsm

return Config
