assert(Config, 'Need a pre-existing Config table!')

local fsm = {}

Config.torque_legs = true

-- Update rate in Hz
fsm.update_rate = 120
-- TODO: Grab from Webots, too...

-- Which FSMs should be enabled?
fsm.enabled = {
	Arm = true,
	Body = true,
	Head = true,
	Motion = true,
	Gripper = true,
	Lidar = true,
}

--SJ: now we can have multiple FSM options
fsm.select = {
	Arm = 'DRCFinal',
	Body = 'DRCFinal', 
	Gripper = 'DRCFinal',
	Head = 'DRCFinal', 
	Motion = 'DRCFinal'
}

-- Custom libraries
fsm.libraries = {
	MotionLib = 'DRCFinal',
	ArmLib = 'DRCFinal', --Now steve's libs are merged into one 
	World = 'DRCFinal'
}

fsm.Arm = {
	-- Idle
	{'armIdle', 'timeout', 'armIdle'},

	--armInitWalk initializes the arms to walk configuration
	--This is done in joint-level, and (hopefully) should work with any initial arm configurations
	{'armIdle', 'init', 'armInitWalk'},

	-- armWalk does nothing (the arm should be in walk configuration)
	{'armInitWalk', 'done', 'armWalk'},
	-- Should have this in case
	{'armWalk', 'teleop', 'armTeleop'},
	{'armWalk', 'teleopraw', 'armTeleopRaw'},

  --armInitManipulation raises the arms to the manipulation configuration
  --should always transition from walk configuration. May work from elsewhere
	{'armWalk', 'ready', 'armManipulation'},

	--When raising is done, arm state remains in armManipulation	
	{'armManipulation', 'init', 'armInitWalk'},
	{'armManipulation', 'pulldoor', 'armPullDoor'},
	{'armManipulation', 'teleop', 'armTeleop'},
	{'armManipulation', 'teleopraw', 'armTeleopRaw'},

	-- Teleop for manipulation if needed
	{'armTeleop', 'init', 'armInitWalk'},
	{'armTeleop', 'ready', 'armManipulation'},
	{'armTeleop', 'teleopraw', 'armTeleopRaw'},

	-- Teleop Raw: For escaping from a bad state
	{'armTeleopRaw', 'teleopraw', 'armTeleopRaw'},
	{'armTeleopRaw', 'teleop', 'armTeleop'},
	{'armTeleopRaw', 'ready', 'armManipulation'},
	{'armTeleopRaw', 'init', 'armInitWalk'},

	-- armPullDoor
	{'armPullDoor', 'done', 'armTeleop'},
	{'armPullDoor', 'ready', 'armManipulation'},
	{'armPullDoor', 'pulldoor', 'armPullDoor'},
	{'armPullDoor', 'teleop', 'armTeleop'},
	{'armPullDoor', 'teleopraw', 'armTeleopRaw'},

	--Old teleop code
	----[[
	{'armWalk', 'teleopold', 'armTeleopSJOLD'},
	{'armTeleopSJOLD', 'done', 'armWalk'},
	--]]

	-- armJacobian is for testing purposes only!
	--[[
	{'armInit', 'jacobian', 'armJacobian'},
	{'armJacobian', 'done', 'armTeleop'},
	{'armReady', 'jacobian', 'armJacobian'},
	{'armJacobian', 'teleopraw', 'armTeleopRaw'},
	{'armJacobian', 'timeout', 'armJacobian'},
	{'armJacobian', 'done', 'armTeleop'},
	{'armJacobian', 'ready', 'armReady'},
	{'armJacobian', 'pulldoor', 'armPullDoor'},
	--]]
}

fsm.Body = {
  {'bodyIdle', 'init', 'bodyInit'},
  --
  {'bodyInit', 'done', 'bodyStop'},  
  --
  {'bodyStop', 'approach', 'bodyApproachMessy'},
  {'bodyApproachMessy', 'done', 'bodyStop'},
  --
  {'bodyStop', 'approachbuggy', 'bodyApproachBuggy'}, --steve's approoach code
  {'bodyApproachBuggy', 'done', 'bodyStop'},
  --
  {'bodyStop', 'stepover1', 'bodyStep'},
  {'bodyStep', 'nextstep', 'bodyStep'},
  {'bodyStep', 'done', 'bodyStop'},
  --

  --take a single, low step (for flat terrain)
	{'bodyStop', 'stepflat', 'bodyStep2'},
  {'bodyStep2', 'done', 'bodyStop'},


}


fsm.Head = {
	{'headIdle', 'init', 'headCenter'},
	--
	{'headCenter', 'teleop', 'headTeleop'},
	{'headCenter', 'trackhand', 'headTrackHand'},
	{'headCenter', 'mesh', 'headMesh'},
	--
	{'headTeleop', 'init', 'headCenter'},
	{'headTeleop', 'trackhand', 'headTrackHand'},
	--
	{'headTrackHand', 'init', 'headCenter'},
	{'headTrackHand', 'teleop', 'headTeleop'},
	--
	{'headMesh', 'init', 'headCenter'},
	{'headMesh', 'done', 'headCenter'},
}

fsm.Gripper = {
	{'gripperIdle', 'init', 'gripperClose'},
	{'gripperIdle', 'open', 'gripperOpen'},
	{'gripperIdle', 'teleop', 'gripperTeleopTorque'},
	{'gripperIdle', 'teleoppos', 'gripperTeleopPosition'},
	--
	{'gripperClose', 'idle', 'gripperIdle'},
	{'gripperClose', 'center', 'gripperCenter'},
	{'gripperClose', 'clench', 'gripperClench'},
	{'gripperClose', 'open', 'gripperOpen'},
	{'gripperClose', 'teleop', 'gripperTeleopTorque'},
	{'gripperClose', 'teleoppos', 'gripperTeleopPosition'},
	--
	{'gripperCenter', 'idle', 'gripperIdle'},
	{'gripperCenter', 'init', 'gripperClose'},
	{'gripperCenter', 'clench', 'gripperClench'},
	{'gripperCenter', 'open', 'gripperOpen'},
	{'gripperCenter', 'teleop', 'gripperTeleopTorque'},
	{'gripperCenter', 'teleoppos', 'gripperTeleopPosition'},
	--
	{'gripperClench', 'idle', 'gripperIdle'},
	{'gripperClench', 'init', 'gripperClose'},
	{'gripperClench', 'center', 'gripperCenter'},
	{'gripperClench', 'open', 'gripperOpen'},
	{'gripperClench', 'teleop', 'gripperTeleopTorque'},
	{'gripperClench', 'teleoppos', 'gripperTeleopPosition'},
	--
	{'gripperOpen', 'idle', 'gripperIdle'},
	{'gripperOpen', 'init', 'gripperClose'},
	{'gripperOpen', 'clench', 'gripperClench'},
	{'gripperOpen', 'center', 'gripperCenter'},
	{'gripperOpen', 'teleop', 'gripperTeleopTorque'},
	{'gripperOpen', 'teleoppos', 'gripperTeleopPosition'},
	--
	{'gripperTeleopTorque', 'idle', 'gripperIdle'},
	{'gripperTeleopTorque', 'init', 'gripperClose'},
	{'gripperTeleopTorque', 'clench', 'gripperClench'},
	{'gripperTeleopTorque', 'teleoppos', 'gripperTeleopPosition'},
	--
	{'gripperTeleopPosition', 'idle', 'gripperIdle'},
	{'gripperTeleopPosition', 'init', 'gripperClose'},
	{'gripperTeleopPosition', 'clench', 'gripperClench'},
	{'gripperTeleopPosition', 'teleop', 'gripperTeleopTorque'},
}

fsm.Lidar = {
	{'lidarIdle', 'pan', 'lidarPan'},
	{'lidarPan', 'switch', 'lidarPan'},
	{'lidarPan', 'stop', 'lidarIdle'},
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

	{'motionStance', 'slowstep', 'motionSlowStep'},
	{'motionSlowStep', 'done', 'motionStance'},


	{'motionStance', 'hybridwalk', 'motionHybridWalkInit'},
	{'motionHybridWalkInit', 'done', 'motionHybridWalk'},
	{'motionHybridWalk', 'done', 'motionHybridWalkEnd'},
	{'motionHybridWalkEnd', 'done', 'motionStance'},

	{'motionStance', 'uninit', 'motionUnInit'},
	{'motionUnInit', 'done', 'motionIdle'},
}

Config.fsm = fsm

return Config
