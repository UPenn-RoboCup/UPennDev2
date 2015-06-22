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
	{'armWalk', 'teleopoldl', 'armTeleopSJOLDL'},
	{'armWalk', 'teleopoldr', 'armTeleopSJOLDR'},
	{'armTeleopSJOLDL', 'done', 'armWalk'},
	{'armTeleopSJOLDR', 'done', 'armWalk'},
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
	{'bodyStop', 'approachbuggy', 'bodyApproachBuggy'},
	{'bodyStop', 'approach', 'bodyApproachMessy'},
	{'bodyStop', 'stepover1', 'bodyStep'},
	{'bodyStop', 'stepflat', 'bodyStepAlign'},
	{'bodyStop', 'stop', 'bodyStop'},
  --
  {'bodyApproachMessy', 'stop', 'bodyStop'},
	{'bodyApproachMessy', 'done', 'bodyStop'},
  --
  {'bodyApproachBuggy', 'stop', 'bodyStop'},
	{'bodyApproachBuggy', 'done', 'bodyStop'},
  --
  {'bodyStep', 'nextstep', 'bodyStep'},
  {'bodyStep', 'done', 'bodyStop'},

  -- Take two slow stops (for precise alignment)
  {'bodyStepAlign', 'done', 'bodyStop'},
}


fsm.Head = {
	{'headIdle', 'init', 'headCenter'},
	{'headIdle', 'teleop', 'headTeleop'},
	--
	{'headCenter', 'done', 'headMesh'},
	{'headCenter', 'trackleft', 'headTrackLeft'},
	{'headCenter', 'trackright', 'headTrackRight'},
	{'headCenter', 'teleop', 'headTeleop'},
	-- Overrides
	{'headTeleop', 'init', 'headCenter'},
	{'headTeleop', 'trackleft', 'headTrackLeft'},
	{'headTeleop', 'trackright', 'headTrackRight'},
	{'headTeleop', 'mesh', 'headMesh'},
	--
	{'headTrackLeft', 'init', 'headCenter'},
	{'headTrackLeft', 'mesh', 'headMesh'},
	{'headTrackLeft', 'trackright', 'headTrackRight'},
	{'headTrackLeft', 'teleop', 'headTeleop'},
	--
	{'headTrackRight', 'init', 'headCenter'},
	{'headTrackRight', 'mesh', 'headMesh'},
	{'headTrackRight', 'trackleft', 'headTrackLeft'},
	{'headTrackRight', 'teleop', 'headTeleop'},
	--
	{'headMesh', 'done', 'headCenter'},
	{'headMesh', 'trackleft', 'headTrackLeft'},
	{'headMesh', 'trackright', 'headTrackRight'},
	{'headMesh', 'init', 'headCenter'},
	{'headMesh', 'teleop', 'headTeleop'},
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
	{'gripperTeleopTorque', 'center', 'gripperCenter'},
	{'gripperTeleopTorque', 'teleoppos', 'gripperTeleopPosition'},
	--
	{'gripperTeleopPosition', 'idle', 'gripperIdle'},
	{'gripperTeleopPosition', 'init', 'gripperClose'},
	{'gripperTeleopPosition', 'clench', 'gripperClench'},
	{'gripperTeleopPosition', 'center', 'gripperCenter'},
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
	{'motionIdle', 'bias', 'motionBiasInit'},
	--
	{'motionBiasInit', 'done', 'motionBiasIdle'},
	{'motionBiasIdle', 'stand', 'motionInit'},
	--
	{'motionInit', 'done', 'motionStance'},
	--
	{'motionUnInit', 'done', 'motionIdle'},
	--
	{'motionStance', 'bias', 'motionBiasInit'},
	{'motionStance', 'uninit', 'motionUnInit'},
	{'motionStance', 'hybridwalk', 'motionHybridWalkInit'},
	{'motionStance', 'preview', 'motionStepPreview'},
	{'motionStance', 'slowstep', 'motionSlowStep'},
	{'motionStance', 'stair', 'motionStepPreviewStair'},
	----- ADD BY HJ JUN 3rd 2015
	{'motionStance', 'getup', 'motionGetupFront'},  
 	{'motionGetupFront', 'done', 'motionInit'},
	-----
	{'motionHybridWalkInit', 'done', 'motionHybridWalk'},
	{'motionHybridWalk', 'done', 'motionHybridWalkEnd'},
	{'motionHybridWalkEnd', 'done', 'motionStance'},
	{'motionHybridWalk', 'stand', 'motionHybridWalkEnd'},
	--
	{'motionStepPreview', 'done', 'motionStance'},
	--
	{'motionStepPreviewStair', 'done', 'motionStance'},
	--
	{'motionSlowStep', 'done', 'motionStance'},


Config.fsm = fsm

return Config
