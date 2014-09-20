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
  Body = 'DRCNew',
  Motion = 'RoboCup'
}


fsm.Lidar = {
  {'lidarIdle', 'pan', 'lidarPan'},
  {'lidarIdle', 'pansingle', 'lidarPanSingle'},
  {'lidarPanSingle', 'done', 'lidarIdle'},
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
  {'bodyStop', 'approach', 'bodyBlockApproach'},
  {'bodyStop', 'footrace', 'bodyFootRace'},


  {'bodyStop', 'stepover', 'bodyStepOver'},
  {'bodyStepOver', 'done', 'bodyStop'},

  {'bodyStepPlace',   'done', 'bodyStop'},
--  {'bodyStepWaypoint',   'done', 'bodyStop'},

  {'bodyBlockApproach', 'done', 'bodyBlockWait'},
--  {'bodyBlockWait', 'done', 'bodyStepOver'},
  {'bodyBlockWait', 'done', 'bodyStop'},


  {'bodyFootRace', 'done', 'bodyStop'},

}

fsm.Arm = {
  {'armIdle', 'timeout', 'armIdle'},
  {'armIdle', 'drive', 'armDrive'},
  {'armIdle', 'init', 'armInit'},

  {'armInit', 'done', 'armPose1'},

  {'armPose1', 'bodyslave', 'armSlave'}, --Full body motion
  
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
  {'motionIdle', 'bias', 'motionBiasInit'},

  {'motionBiasInit', 'done', 'motionBiasIdle'}, 
  {'motionBiasIdle', 'stand', 'motionInit'}, 

  {'motionInit', 'done', 'motionStance'},

  {'motionStance', 'bias', 'motionBiasInit'},
  {'motionStance', 'preview', 'motionStepPreview'},
  {'motionStance', 'kick', 'motionKick'},
  {'motionStance', 'done_step', 'motionHybridWalkKick'},

  {'motionStance', 'getup', 'motionGetupFront'},

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

fsm.dqNeckLimit = {
  60 * DEG_TO_RAD, 60 * DEG_TO_RAD
}

fsm.dqNeckLimit = {
  60 * DEG_TO_RAD, 60 * DEG_TO_RAD
}

fsm.headScan = {
  pitch0 = 30 * DEG_TO_RAD,
  pitchMag = 20 * DEG_TO_RAD,
  --yawMag = 80 * DEG_TO_RAD,
  yawMag = 40 * DEG_TO_RAD,
  tScan = 5, --sec
}

--HeadReady
fsm.headReady = {
  dist = 3
}

--HeadTrack
fsm.headTrack = {
  tLost = 5,
  timeout = 6,
  dist_th = 0.5,
}

--HeadLookGoal: Look up to see the goal
fsm.headLookGoal = {
  yawSweep = 80*DEG_TO_RAD,
}

--HeadSweep: Look around to find the goal
fsm.headSweep = {
  tScan = 2.0,
  tWait = 0.25,
}

fsm.headObstacleScan = {
  yawMag = 55*DEG_TO_RAD,
  pitchUp = 25*DEG_TO_RAD,
  pitchDown = 35*DEG_TO_RAD,
}

fsm.bodyRobocupFollow = {
  th_lfoot = 0.001,
  th_rfoot = 0.001,
  th_dist = 0.08,  --TODO
}

fsm.bodyRobocupApproach = {
  target={0.30,0.12} ,
  th = {0.34, 0.02}
}

if IS_WEBOTS then
  fsm.headScan.tScan = 16
  fsm.bodyRobocupFollow.th_dist = 0.2
end

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










Config.fsm.headTrack.timeout = 3  
Config.fsm.dqNeckLimit ={40*DEG_TO_RAD, 180*DEG_TO_RAD}

Config.approachTargetX = {
  0.45, --for kick 0 (walkkick)
  0.28, --for kick 1 (st kick)
  0.35  --for kick 2 (weak walkkick)
}

--Config.disable_kick = true
Config.disable_kick = false

Config.use_angle_localization = true
Config.demo = false
--Config.demo = true

if IS_WEBOTS then 
--  Config.use_gps_pose = false
  Config.use_gps_pose = true
  Config.use_localhost = true
end


--  Config.approachTargetY= {-0.07,0.05}  --L/R aiming offsets
Config.approachTargetY= {-0.07,0.02}  --L/R aiming offsets

Config.ballX_threshold1 = -1.5 --The threshold we use walkkick
Config.ballX_threshold2 = 0.5 --The threshold we start using strong kick


Config.disable_ball_when_lookup = true

Config.maxStepApproachTh = 0.30
Config.maxStepApproach1 = 0.10
Config.maxStepApproach2 = 0.06


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
