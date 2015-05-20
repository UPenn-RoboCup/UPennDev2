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
	ArmLib = 'DRCFinal',
	World = 'DRCFinal'
}

if IS_STEVE then
	--fsm.libraries.MotionLib = 'Steve'
	fsm.libraries.ArmLib = 'Steve'
	fsm.select.Arm = 'Steve'
	if IS_WEBOTS then
		Config.testfile = 'test_teleop'
	end
end


fsm.Arm = {
	-- Idle
	{'armIdle', 'timeout', 'armIdle'},
	{'armIdle', 'init', 'armInit'},
	-- Init
	--{'armInit', 'timeout', 'armInit'},
	--{'armInit', 'done', 'armInit'},
	{'armInit', 'ready', 'armReady'},
	{'armInit', 'teleopraw', 'armTeleopRaw'},
	-- Test the jacobian
	--[[
	{'armInit', 'jacobian', 'armJacobian'},
	{'armJacobian', 'done', 'armTeleop'},
	--]]

	-- Ready pose (for manipulating)
	--{'armReady', 'timeout', 'armReady'},
	{'armReady', 'teleop', 'armTeleop'},
	{'armReady', 'teleopraw', 'armTeleopRaw'},
	{'armReady', 'init', 'armInit'},
	{'armReady', 'jacobian', 'armJacobian'},
	{'armReady', 'pulldoor', 'armPullDoor'},
	-- Teleop
	{'armTeleop', 'init', 'armInit'},
	--{'armTeleop', 'done', 'armTeleop'},
	{'armTeleop', 'teleop', 'armTeleop'},
	{'armTeleop', 'ready', 'armReady'},
	{'armTeleop', 'teleopraw', 'armTeleopRaw'},
	-- Teleop Raw
	{'armTeleopRaw', 'init', 'armInit'},
	{'armTeleopRaw', 'teleopraw', 'armTeleopRaw'},
	{'armTeleopRaw', 'ready', 'armReady'},
	{'armTeleopRaw', 'teleop', 'armTeleop'},
	-- armJacobian is for testing purposes only!
	{'armJacobian', 'teleopraw', 'armTeleopRaw'},
	{'armJacobian', 'timeout', 'armJacobian'},
	{'armJacobian', 'done', 'armTeleop'},
	{'armJacobian', 'ready', 'armReady'},
	{'armJacobian', 'pulldoor', 'armPullDoor'},
	-- armPullDoor
	{'armPullDoor', 'teleopraw', 'armTeleopRaw'},
	{'armPullDoor', 'done', 'armTeleop'},
	{'armPullDoor', 'ready', 'armReady'},
	{'armPullDoor', 'pulldoor', 'armPullDoor'},
}

if fsm.libraries.ArmLib == 'DRCFinal' then
	fsm.select.Arm = 'DRCFinal'
	fsm.Arm = {
		{'armIdle', 'init', 'armInit'},
		{'armInit', 'done', 'armPose1'},

		{'armPose1', 'teleop', 'armTeleop'},	
		{'armTeleop', 'done', 'armPose1'},	
	}
end


fsm.Body = {
  {'bodyIdle', 'init', 'bodyInit'},
  --
  {'bodyInit', 'done', 'bodyStop'},  
  --
  {'bodyStop', 'approach', 'bodyApproach2'},
  {'bodyApproach2', 'done', 'bodyStop'},
  --
  {'bodyStop', 'approach2', 'bodyApproach'}, --steve's approoach code
  {'bodyApproach', 'done', 'bodyStop'},
  --
  {'bodyStop', 'stepover1', 'bodyStep'},
  {'bodyStep', 'nextstep', 'bodyStep'},
  {'bodyStep', 'done', 'bodyStop'},
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
	{'gripperIdle', 'init', 'gripperCenter'},
	{'gripperIdle', 'teleop', 'gripperTeleop'},	
	--
	{'gripperCenter', 'idle', 'gripperIdle'},
	{'gripperCenter', 'teleop', 'gripperTeleop'},
	--
	{'gripperTeleop', 'idle', 'gripperIdle'},
	{'gripperTeleop', 'init', 'gripperCenter'},
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

	{'motionStance', 'hybridwalk', 'motionHybridWalkInit'},
	{'motionHybridWalkInit', 'done', 'motionHybridWalk'},
	{'motionHybridWalk', 'done', 'motionHybridWalkEnd'},
	{'motionHybridWalkEnd', 'done', 'motionStance'},

	{'motionStance', 'uninit', 'motionUnInit'},
	{'motionUnInit', 'done', 'motionIdle'},
}

Config.fsm = fsm

return Config
