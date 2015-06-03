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

fsm.Body = {
  {'bodyIdle', 'init', 'bodyInit'},
  --
  {'bodyInit', 'done', 'bodyStop'},
	--
	{'bodyStop', 'approachbuggy', 'bodyApproachBuggy'},
	{'bodyStop', 'approach', 'bodyApproachMessy'},
	
	{'bodyStop', 'stepflat', 'bodyStepAlign'},
	{'bodyStop', 'stop', 'bodyStop'},
  --
  {'bodyApproachMessy', 'stop', 'bodyStop'},
	{'bodyApproachMessy', 'done', 'bodyStop'},
  --
  {'bodyApproachBuggy', 'stop', 'bodyStop'},
	{'bodyApproachBuggy', 'done', 'bodyStop'},
  --
  --these should NEVER called with mistake at all
  {'bodyStop', 'stairclimb', 'bodyStep'},
  {'bodyStep', 'done', 'bodyStop'},
  -- Take two slow stops (for precise alignment)
  {'bodyStepAlign', 'done', 'bodyStop'},


--Driving stuff
  {'bodyStop', 'driveready', 'bodyDriveready'},   --untorques leg and arm, rotate the head back, centers lidar
  {'bodyDriveready', 'drive', 'bodyDrive'}, -- torques all servos and enable foot and arm control

  {'bodyDrive', 'driveready', 'bodyUndrive'}, -- untorques arm and leg for egress
  {'bodyUndrive', 'init', 'bodyInit'}, --re-inits leg

}

-- Anything can fall!
local allbody = {}
for i,v in ipairs(fsm.Body) do allbody[v[1]] = true end
for k, v in pairs(allbody) do table.insert(fsm.Body, {k, 'fall', 'bodyFall'}) end
-- Maybe just restart state_wizard at this point...



fsm.Head = {
	{'headIdle', 'init', 'headCenter'},
	{'headIdle', 'teleop', 'headTeleop'},
	--
	{'headCenter', 'trackleft', 'headTrackLeft'},
	{'headCenter', 'trackright', 'headTrackRight'},
	{'headCenter', 'mesh', 'headMesh'},
	{'headCenter', 'teleop', 'headTeleop'},
	{'headCenter', 'teleopik', 'headTeleopIK'},
	--
	{'headTeleop', 'init', 'headCenter'},
	{'headTeleop', 'trackleft', 'headTrackLeft'},
	{'headTeleop', 'trackright', 'headTrackRight'},
	{'headTeleop', 'mesh', 'headMesh'},
	{'headTeleop', 'teleopik', 'headTeleopIK'},
	--
	{'headTeleopIK', 'init', 'headCenter'},
	{'headTeleopIK', 'trackleft', 'headTrackLeft'},
	{'headTeleopIK', 'trackright', 'headTrackRight'},
	{'headTeleopIK', 'mesh', 'headMesh'},
	{'headTeleopIK', 'teleop', 'headTeleop'},
	--
	{'headTrackLeft', 'init', 'headCenter'},
	{'headTrackLeft', 'mesh', 'headMesh'},
	{'headTrackLeft', 'trackright', 'headTrackRight'},
	{'headTrackLeft', 'teleop', 'headTeleop'},
	{'headTrackLeft', 'teleopik', 'headTeleopIK'},
	--
	{'headTrackRight', 'init', 'headCenter'},
	{'headTrackRight', 'mesh', 'headMesh'},
	{'headTrackRight', 'trackleft', 'headTrackLeft'},
	{'headTrackRight', 'teleop', 'headTeleop'},
	{'headTrackRight', 'teleopik', 'headTeleopIK'},
	--
	{'headMesh', 'trackleft', 'headTrackLeft'},
	{'headMesh', 'trackright', 'headTrackRight'},
	{'headMesh', 'init', 'headCenter'},
	{'headMesh', 'teleop', 'headTeleop'},
	{'headMesh', 'teleopik', 'headTeleopIK'},

--Driving stuff




	{'headCenter', 'drive', 'headDrive'}, --go to 180 deg rotated position
	{'headTeleop', 'drive', 'headDrive'},
	{'headTrackLeft', 'drive', 'headDrive'},
	{'headTrackRight', 'drive', 'headDrive'},

	{'headDrive', 'undrive', 'headCenter'}, --unscrew the head safelty
}












fsm.Arm = {
	-- Idle
	{'armIdle', 'timeout', 'armIdle'},

	--armInitWalk initializes the arms to walk configuration
	--This is done in joint-level, and (hopefully) should work with any initial arm configurations
	{'armIdle', 'init', 'armInitWalk'},
	{'armIdle', 'bias', 'armInitBias'},
	{'armIdle', 'ready', 'armManipulation'},

	{'armWalk', 'bias', 'armInitBias'},
	{'armInitBias', 'done', 'armWalk'},

	-- armWalk does nothing (the arm should be in walk configuration)
	{'armInitWalk', 'done', 'armWalk'},

	-- From the walk state
	{'armWalk', 'pushdoor', 'armPushDoorUp'},
	{'armWalk', 'ready', 'armManipulation'},
	{'armWalk', 'teleop', 'armTeleop'},
	{'armWalk', 'teleopraw', 'armTeleopRaw'},

	--Old teleop code
	----[[
	{'armWalk', 'teleopoldl', 'armTeleopSJOLDL'},
	{'armWalk', 'teleopoldr', 'armTeleopSJOLDR'},
	{'armTeleopSJOLDL', 'done', 'armWalk'},
	{'armTeleopSJOLDR', 'done', 'armWalk'},
	--]]
	
	-- Teleop IK level
	{'armTeleop', 'init', 'armInitWalk'},
	{'armTeleop', 'teleopraw', 'armTeleopRaw'},
	{'armTeleop', 'teleop', 'armTeleop'},

	-- Teleop Joint level
	{'armTeleopRaw', 'init', 'armInitWalk'},
	{'armTeleopRaw', 'teleopraw', 'armTeleopRaw'},
	{'armTeleopRaw', 'teleop', 'armTeleop'},

	--When raising is done, arm state remains in armManipulation	
	{'armManipulation', 'init', 'armInitWalk'},
	{'armManipulation', 'ready', 'armManipulation'},
	{'armManipulation', 'pushdoor', 'armPushDoorDown'},
	{'armManipulation', 'drill', 'armDrill'},
	{'armManipulation', 'shower', 'armShower'},
	{'armManipulation', 'valve', 'armValve'},
	--
	{'armTeleop', 'ready', 'armManipulation'},
	{'armTeleopRaw', 'ready', 'armManipulation'},
	{'armManipulation', 'teleop', 'armTeleop'},
	{'armManipulation', 'teleopraw', 'armTeleopRaw'},

	-- PushDoor positioning
	{'armPushDoorDown', 'ready', 'armManipulation'},
	{'armPushDoorDown', 'done', 'armManipulation'},
	--
	{'armTeleop', 'pushdoor', 'armPushDoorDown'},
	{'armTeleopRaw', 'pushdoor', 'armPushDoorDown'},
	{'armPushDoorDown', 'teleop', 'armTeleop'},
	{'armPushDoorDown', 'teleopraw', 'armTeleopRaw'},

	-- PushDoor positioning
	{'armWalk', 'pushdoor', 'armPushDoorUp'},
	{'armPushDoorUp', 'done', 'armManipulation'},
	--
	{'armTeleop', 'pushdoor', 'armPushDoorUp'},
	{'armTeleopRaw', 'pushdoor', 'armPushDoorUp'},
	{'armPushDoorUp', 'teleop', 'armTeleop'},
	{'armPushDoorUp', 'teleopraw', 'armTeleopRaw'},

	-- Valve positioning
	{'armWalk', 'valve', 'armValve'},
	--{'armValve', 'done', 'armTeleop'},
	{'armValve', 'done', 'armManipulation'},
	{'armValve', 'ready', 'armManipulation'},
	--
	{'armTeleop', 'valve', 'armValve'},
	{'armTeleopRaw', 'valve', 'armValve'},
	{'armValve', 'teleop', 'armTeleop'},
	{'armValve', 'teleopraw', 'armTeleopRaw'},

	-- Drill positioning
	{'armDrill', 'done', 'armTeleop'},
	{'armDrill', 'ready', 'armManipulation'},
	--
	{'armTeleop', 'drill', 'armDrill'},
	{'armTeleopRaw', 'drill', 'armDrill'},
	{'armDrill', 'teleop', 'armTeleop'},
	{'armDrill', 'teleopraw', 'armTeleopRaw'},

	-- Shower positioning
	{'armShower', 'done', 'armTeleop'},
	{'armShower', 'ready', 'armManipulation'},
	--
	{'armTeleop', 'shower', 'armShower'},
	{'armTeleopRaw', 'shower', 'armShower'},
	{'armShower', 'teleop', 'armTeleop'},
	{'armShower', 'teleopraw', 'armTeleopRaw'},

	-- armPullDoor
	--[[
	{'armPullDoor', 'done', 'armTeleop'},
	{'armPullDoor', 'ready', 'armManipulation'},
	{'armPullDoor', 'pulldoor', 'armPullDoor'},
	{'armPullDoor', 'teleop', 'armTeleop'},
	{'armPullDoor', 'teleopraw', 'armTeleopRaw'},
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



	{'armWalk', 'driveready', 'armDriveready'},
	{'armDriveready', 'drive', 'armDrive'},
	{'armDrive', 'undrive', 'armUndrive'},
	{'armUndrive', 'init', 'armInitWalk'},

}




fsm.Gripper = {
	{'gripperIdle', 'close', 'gripperClose'},
	{'gripperIdle', 'open', 'gripperOpen'},
	{'gripperIdle', 'center', 'gripperCenter'},
	{'gripperIdle', 'clench', 'gripperClench'},
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
	{'gripperCenter', 'close', 'gripperClose'},
	{'gripperCenter', 'clench', 'gripperClench'},
	{'gripperCenter', 'open', 'gripperOpen'},
	{'gripperCenter', 'teleop', 'gripperTeleopTorque'},
	{'gripperCenter', 'teleoppos', 'gripperTeleopPosition'},
	--
	{'gripperClench', 'idle', 'gripperIdle'},
	{'gripperClench', 'close', 'gripperClose'},
	{'gripperClench', 'center', 'gripperCenter'},
	{'gripperClench', 'open', 'gripperOpen'},
	{'gripperClench', 'teleop', 'gripperTeleopTorque'},
	{'gripperClench', 'teleoppos', 'gripperTeleopPosition'},
	--
	{'gripperOpen', 'idle', 'gripperIdle'},
	{'gripperOpen', 'close', 'gripperClose'},
	{'gripperOpen', 'clench', 'gripperClench'},
	{'gripperOpen', 'center', 'gripperCenter'},
	{'gripperOpen', 'teleop', 'gripperTeleopTorque'},
	{'gripperOpen', 'teleoppos', 'gripperTeleopPosition'},
	--
	{'gripperTeleopTorque', 'idle', 'gripperIdle'},
	{'gripperTeleopTorque', 'close', 'gripperClose'},
	{'gripperTeleopTorque', 'clench', 'gripperClench'},
	{'gripperTeleopTorque', 'open', 'gripperOpen'},
	{'gripperTeleopTorque', 'center', 'gripperCenter'},
	{'gripperTeleopTorque', 'teleoppos', 'gripperTeleopPosition'},
	--
	{'gripperTeleopPosition', 'idle', 'gripperIdle'},
	{'gripperTeleopPosition', 'close', 'gripperClose'},
	{'gripperTeleopPosition', 'clench', 'gripperClench'},
	{'gripperTeleopPosition', 'open', 'gripperOpen'},
	{'gripperTeleopPosition', 'center', 'gripperCenter'},
	{'gripperTeleopPosition', 'teleop', 'gripperTeleopTorque'},
}

fsm.Lidar = {
	{'lidarIdle', 'pan', 'lidarPan'},
	{'lidarPan', 'switch', 'lidarPan'},
	{'lidarPan', 'stop', 'lidarIdle'},

	--while driving chest lidar should be keep centered
	{'lidarPan', 'drive', 'lidarDrive'},
	{'lidarDrive', 'pan', 'lidarPan'},
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
	--
	{'motionHybridWalkInit', 'done', 'motionHybridWalk'},
	{'motionHybridWalk', 'done', 'motionHybridWalkEnd'},
	{'motionHybridWalkEnd', 'done', 'motionStance'},
	--
	{'motionStepPreview', 'done', 'motionStance'},
	--
	{'motionStepPreviewStair', 'done', 'motionStance'},
	--
	{'motionSlowStep', 'done', 'motionStance'},





	--DRIVE
	{'motionStance', 'driveready', 'motionDriveready'}, --untorque lower body
	{'motionDriveready', 'drive', 'motionDrive'}, --torque the body, enable foot control
	
	{'motionDrive', 'undrive', 'motionUndrive'}, --untorque lower body again
	{'motionUndrive', 'stand', 'motionInit'}, --torque all the legs and make the robot stand up


}

Config.fsm = fsm

return Config
