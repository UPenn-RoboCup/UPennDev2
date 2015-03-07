assert(Config, 'Need a pre-existing Config table!')

local fsm = {}

-- Do we disable FSMs?
fsm.disabled = false

-- Do we disable Kick?
--fsm.disable_kick = true
fsm.disable_kick = false

Config.torque_legs = true

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

  {'bodyStop', 'uninit', 'bodyUnInit'},
  {'bodyUnInit', 'done', 'bodyIdle'},

  {'bodyStop', 'approach', 'bodyApproach'},
  {'bodyApproach', 'done', 'bodyStop'},  

--[[
  {'bodyStop', 'stepinplace', 'bodyStepPlace'},
  {'bodyStepPlace',   'done', 'bodyStop'},

  {'bodyStop', 'stepover1', 'bodyStep'},
  {'bodyStep', 'nextstep', 'bodyStep'},
  {'bodyStep', 'done', 'bodyStop'},
--]]
}


--Cut down arm FSM for now

fsm.Arm = {
  {'armIdle', 'timeout', 'armIdle'},
  {'armIdle', 'init', 'armInit'},
  {'armInit', 'done', 'armPose1'},

  {'armPose1', 'teleop', 'armTeleop'},
  {'armPose1', 'toolgrab', 'armToolGrip'},

--  {'armPose1', 'pushdoorgrab', 'armPushDoorSideGrip'},
--  {'armPose1', 'doorgrab', 'armPullDoorSideGrip'},


  {'armToolGrip', 'done', 'armPose1'},
--  {'armToolGrip', 'hold', 'armToolHold'},
  {'armTeleop', 'done', 'armPose1'},


  {'armPose1', 'uninit', 'armUnInit'},
  {'armToolGrip', 'uninit', 'armUnInit'},
  {'armTeleop', 'uninit', 'armUnInit'},

  {'armUnInit', 'done', 'armIdle'},
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
  {'motionStepPreviewStair', 'done', 'motionStance'},

  {'motionStance', 'hybridwalk', 'motionHybridWalkInit'},
  {'motionHybridWalkInit', 'done', 'motionHybridWalk'},
  {'motionHybridWalk', 'done', 'motionHybridWalkEnd'},
  {'motionHybridWalkEnd', 'done', 'motionStance'},

  {'motionStance', 'uninit', 'motionUnInit'},
  {'motionUnInit', 'done', 'motionIdle'},

}

fsm.dqNeckLimit = {
  60 * DEG_TO_RAD, 60 * DEG_TO_RAD
}

Config.fsm = fsm

return Config
