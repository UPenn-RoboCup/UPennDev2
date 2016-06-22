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
	Body = 'RoboCup2015', 
	Gripper = 'DRCFinal',
	Head = 'DRCFinal', 
	Motion = 'RoboCup2015'
}

-- Custom libraries
fsm.libraries = {
	MotionLib = 'DRCFinal',
	ArmLib = 'DRCFinal', --Now steve's libs are merged into one 
	World = 'DRCFinal'
}


------------------------------------------------------------------
---- DRC Body State Code

--[[
fsm.Body = {
  {'bodyIdle', 'init', 'bodyInit'},
  --
  {'bodyInit', 'done', 'bodyStop'},
	--
--	{'bodyStop', 'approachbuggy', 'bodyApproachBuggy'},
	
--	{'bodyStop', 'stepflat', 'bodyStepAlign'},

	{'bodyStop', 'approach', 'bodyApproachMessy'},
	{'bodyStop', 'stepflat', 'bodyApproachMessy'},

	{'bodyStop', 'stop', 'bodyStop'},
  --
  {'bodyApproachMessy', 'stop', 'bodyStop'},
	{'bodyApproachMessy', 'done', 'bodyStop'},
  --
  {'bodyApproachBuggy', 'stop', 'bodyStop'},
	{'bodyApproachBuggy', 'done', 'bodyStop'},
  --


  --these should NEVER called with mistake at all
  --FOR DAY 1
--
  {'bodyStop', 'stepover1', 'bodyStep'},
  {'bodyStep', 'done', 'bodyStop'},
  {'bodyStep', 'nextstep', 'bodyStep'},

  {'bodyStop', 'stairclimb', 'bodyStepPiecewise'},
  {'bodyStepPiecewise', 'done', 'bodyStop'},
  {'bodyStepPiecewise', 'nextstep', 'bodyStepPiecewise'},
--

  -- Take two slow stops (for precise alignment)
--  {'bodyStepAlign', 'done', 'bodyStop'},


--Driving stuff
  {'bodyStop', 'driveready', 'bodyDriveready'},   --untorques leg and arm, rotate the head back, centers lidar
  {'bodyDriveready', 'drive', 'bodyDrive'}, -- torques all servos and enable foot and arm control
  {'bodyDrive', 'driveready', 'bodyDriveready'}, -- untorques arm and leg for egress
  {'bodyDriveready', 'init', 'bodyInit'}, --re-inits leg


  --SOFT E-stop handling
  {'bodyInit', 'estop', 'bodyEStop'},
  {'bodyStop', 'estop', 'bodyEStop'},
  {'bodyApproachMessy', 'estop', 'bodyEStop'},
  {'bodyStep', 'estop', 'bodyEStop'},
  {'bodyStepAlign', 'estop', 'bodyEStop'},
  {'bodyDrive', 'estop', 'bodyEStop'}, -- untorques arm and leg for egress
  {'bodyDriveready', 'estop', 'bodyEStop'}, -- untorques arm and leg for egress
  {'bodyUndrive', 'estop', 'bodyEStop'}, --re-inits leg
  {'bodyEStop', 'done', 'bodyInit'},
}

]]--
--------------------------------------------------------------------


--------------------------------------------------------------------
-- Robocup 

fsm.Body = {
	{'bodyIdle', 'init', 'bodyInit'},
	{'bodyInit', 'done', 'bodyStop'},

	--  {'bodyStop', 'footrace', 'bodyFootRace'},
	--  {'bodyFootRace', 'done', 'bodyStop'},

	{'bodyStop', 'kick', 'bodyRobocupKick'},
	--  {'bodyStop', 'play', 'bodyRobocupIdle'},
	--  {'bodyStop', 'goalie', 'bodyRobocupGoalieIdle'},
	


------------------------------------------------------------------
	--{'bodyStop', 'approach', 'bodyRobocupApproach'},


	{'bodyStop', 'approach', 'bodyApproachMessy'},
	{'bodyStop', 'stepflat', 'bodyApproachMessy'},

	{'bodyStop', 'stop', 'bodyStop'},
	--
	{'bodyApproachMessy', 'stop', 'bodyStop'},
	{'bodyApproachMessy', 'done', 'bodyStop'},
	--

	{'bodyStop', 'stop', 'bodyStop'},   --jinwook

--------------------------------------------------------------------


	{'bodyStop', 'play', 'bodyRobocupIdle'},
	{'bodyStop', 'goalie', 'bodyRobocupGoalieIdle'},

	{'bodyStop', 'uninit', 'bodyUninit'},
	{'bodyUninit', 'done', 'bodyIdle'},


	{'bodyRobocupIdle', 'spin', 'bodyRobocupSpin'},
	{'bodyRobocupSpin', 'ballfound', 'bodyRobocupFollow'},


	{'bodyRobocupIdle', 'timeout', 'bodyRobocupIdle'},
	{'bodyRobocupIdle', 'ballfound', 'bodyRobocupFollow'},
	{'bodyRobocupIdle','stop','bodyStop'},
	{'bodyRobocupIdle','goalie','bodyRobocupGoalieIdle'},

	{'bodyRobocupApproach', 'done', 'bodyRobocupKick'},
	{'bodyRobocupApproach', 'ballfar', 'bodyRobocupFollow'},
	{'bodyRobocupApproach','stop','bodyStop'},

	{'bodyRobocupFollow', 'done', 'bodyRobocupIdle'},
	{'bodyRobocupFollow', 'timeout', 'bodyRobocupFollow'},
	{'bodyRobocupFollow', 'ballclose', 'bodyRobocupApproach'},
	{'bodyRobocupFollow','stop','bodyStop'},

	{'bodyRobocupKick', 'done', 'bodyRobocupIdle'},
	{'bodyRobocupKick', 'testdone', 'bodyStop'},
	--
	{'bodyRobocupGoalieIdle', 'attacker', 'bodyRobocupIdle'},
	{'bodyRobocupGoalieIdle', 'stop', 'bodyStop'},
	{'bodyRobocupGoalieIdle', 'position', 'bodyRobocupGoaliePosition'},
	{'bodyRobocupGoalieIdle', 'timeout', 'bodyRobocupGoalieIdle'},
	--
	{'bodyRobocupGoaliePosition', 'timeout', 'bodyRobocupGoalieDone'},
	{'bodyRobocupGoaliePosition', 'idle', 'bodyRobocupGoalieIdle'},
	{'bodyRobocupGoaliePosition', 'done', 'bodyRobocupGoalieDone'},
	{'bodyRobocupGoaliePosition', 'stop', 'bodyStop'},
	{'bodyRobocupGoaliePosition', 'attacker', 'bodyRobocupIdle'},
	--
	{'bodyRobocupGoalieDone', 'stop', 'bodyStop'},
	{'bodyRobocupGoalieDone', 'attacker', 'bodyRobocupIdle'},

}

--------------------------------------------------------------------

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
	{'headDrive', 'init', 'headCenter'}, --unscrew the head safelty

--E-stop handling
	{'headCenter', 'estop', 'headIdle'}, --go to 180 deg rotated position
	{'headTeleop', 'estop', 'headIdle'},
	{'headTrackLeft', 'estop', 'headIdle'},
	{'headTrackRight', 'estop', 'headIdle'},
	{'headDrive', 'estop', 'headIdle'}, --unscrew the head safelty

}



fsm.Arm = {


	--RRT
	{'armIdle', 'RRTbasedmove', 'armRRTbasedmove'},  --jwhuh
	{'armTeleop','RRTbasedmove', 'armRRTbasedmove'},  --jwhuh
	--	{'armManipulation','RRTbasedmove','armRRTbasedmove'} --jwhuh
	{'armTeleopRaw', 'RRTbasedmove', 'armRRTbasedmove'},  --jwhuh
	{'armRRTbasedmove', 'teleopraw', 'armTeleopRaw'},  --jwhuh
	{'armRRTbasedmove', 'done', 'armTeleop'},  --jwhuh
	{'armRRTmoveReady', 'done', 'armRRTmoveReady'},  --jwhuh
	{'armManipulation', 'RRTbasedmove', 'armRRTbasedmove'},  --jwhuh
	{'armRRTbasedmove', 'ready', 'armManipulation'},  --jwhuh
--	{'armWalk', 'RRTbasedmove', 'armRRTbasedmove'},  --jwhuh




	-- Idle
	{'armIdle', 'timeout', 'armIdle'},
	{'armIdle', 'teleopraw', 'armTeleopRaw'},
	{'armIdle', 'teleop', 'armTeleop'},
--
	{'armTeleopRaw', 'idle', 'armIdle'},
	{'armTeleop', 'idle', 'armIdle'},


	--This should be called JUST ONCE at the beginning
	{'armIdle', 'init', 'armInitFirst'},   --armInitFirst
--	{'armIdle', 'init', 'armInitWalk'},


	{'armIdle', 'bias', 'armInitBias'},
	{'armIdle', 'ready', 'armManipulation'},

	{'armWalk', 'bias', 'armInitBias'},
	{'armInitBias', 'done', 'armWalk'},


--armInitWalk initializes the arms to walk configuration
--This is done in joint-level, and (hopefully) should work with any initial arm configurations


	-- armWalk does nothing (the arm should be in walk configuration)
	{'armInitWalk', 'done', 'armWalk'},
	{'armInitFirst', 'done', 'armWalk'},

	-- From the walk state
	--{'armWalk', 'pushdoor', 'armPushDoorUp'},
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


	--NEW transitions added from SJ
	{'armWalk', 'pushdoordown', 'armPushDoorDown'},  --THIS WORKS
	{'armWalk', 'valve', 'armValve'}, --THIS works too!
	{'armPushDoorDown', 'init', 'armInitWalk'}, --this kinda works too

	{'armWalk', 'plug', 'armPlug'},
	{'armPlug', 'init', 'armInitWalk'},





	
	-- Teleop IK level
	{'armTeleop', 'init', 'armInitWalk'},
	{'armTeleop', 'teleopraw', 'armTeleopRaw'},
	{'armTeleop', 'teleop', 'armTeleop'},

	-- Teleop Joint level
	{'armTeleopRaw', 'init', 'armInitWalk'},
	{'armTeleopRaw', 'teleopraw', 'armTeleopRaw'},
	{'armTeleopRaw', 'teleop', 'armTeleop'},



	{'armTeleop', 'valverotate', 'armValveRotate'},
	{'armTeleopRaw', 'valverotate', 'armValveRotate'},

	{'armValveRotate', 'teleop', 'armTeleop'},
	{'armValveRotate', 'teleopraw', 'armTeleopRaw'},


	--When raising is done, arm state remains in armManipulation	
	{'armManipulation', 'init', 'armInitWalk'},
	{'armManipulation', 'ready', 'armManipulation'},
	{'armManipulation', 'pushdoordown', 'armPushDoorDown'},
	{'armManipulation', 'drill', 'armDrill'},
	--{'armManipulation', 'shower', 'armShower'},
	{'armManipulation', 'valve', 'armValve'},
	{'armManipulation', 'plug', 'armPlug'},
	--
	{'armTeleop', 'ready', 'armManipulation'},
	{'armTeleopRaw', 'ready', 'armManipulation'},
	{'armManipulation', 'teleop', 'armTeleop'},
	{'armManipulation', 'teleopraw', 'armTeleopRaw'},

	-- PushDoor positioning
	{'armPushDoorDown', 'ready', 'armManipulation'},
	--{'armPushDoorDown', 'done', 'armManipulation'},
	--
	{'armTeleop', 'pushdoordown', 'armPushDoorDown'},
	{'armTeleopRaw', 'pushdoordown', 'armPushDoorDown'},
	{'armPushDoorDown', 'teleop', 'armTeleop'},
	{'armPushDoorDown', 'teleopraw', 'armTeleopRaw'},

	-- Down arm positioning ( just the left for now
	{'armWalk', 'down', 'armDown'},
	--
	{'armTeleop', 'down', 'armDown'},
	{'armTeleopRaw', 'down', 'armDown'},
	{'armDown', 'teleop', 'armTeleop'},
	{'armDown', 'teleopraw', 'armTeleopRaw'},

	-- Plug positioning
	{'armWalk', 'plug', 'armPlug'},
	--
	{'armTeleop', 'plug', 'armPlug'},
	{'armTeleopRaw', 'plug', 'armPlug'},
	{'armPlug', 'teleop', 'armTeleop'},
	{'armPlug', 'teleopraw', 'armTeleopRaw'},

	-- Valve positioning
	{'armWalk', 'valve', 'armValve'},
	{'armValve', 'done', 'armTeleop'},
	--
	{'armTeleop', 'valve', 'armValve'},
	{'armTeleopRaw', 'valve', 'armValve'},
	{'armValve', 'teleop', 'armTeleop'},
	{'armValve', 'teleopraw', 'armTeleopRaw'},

	-- Valve turning
	--[[
	{'armChopstickTurn', 'done', 'armTeleop'},
	--
	{'armTeleop', 'turn', 'armChopstickTurn'},
	{'armTeleopRaw', 'turn', 'armChopstickTurn'},
	{'armChopstickTurn', 'teleop', 'armTeleop'},
	{'armChopstickTurn', 'teleopraw', 'armTeleopRaw'},
	--]]

	-- Drill positioning
	--{'armDrill', 'done', 'armCarry'},
	{'armDrill', 'drillright', 'armDrillRight'},
	{'armDrill', 'drillleft', 'armDrillLeft'},
	{'armDrill', 'ready', 'armManipulation'},
	--
	{'armDrill', 'teleop', 'armTeleop'},
	{'armDrill', 'teleopraw', 'armTeleopRaw'},

	-- Drill positioning (right)
	--[[
	{'armDrillRight', 'done', 'armCarry'},
	{'armDrillRight', 'ready', 'armManipulation'},
	--
	{'armTeleop', 'drill', 'armDrillRight'},
	{'armTeleopRaw', 'drill', 'armDrillRight'},
	{'armDrillRight', 'teleop', 'armTeleop'},
	{'armDrillRight', 'teleopraw', 'armTeleopRaw'},

	-- Drill positioning (left)
	{'armDrillLeft', 'done', 'armCarry'},
	{'armDrillLeft', 'ready', 'armManipulation'},
	--
	{'armTeleop', 'drill', 'armDrillLeft'},
	{'armTeleopRaw', 'drill', 'armDrillLeft'},
	{'armDrillLeft', 'teleop', 'armTeleop'},
	{'armDrillLeft', 'teleopraw', 'armTeleopRaw'},
	--]]

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


	{'armTeleopRaw', 'driveready', 'armDriveready'},
	{'armWalk', 'driveready', 'armDriveready'},
	{'armDriveready', 'drive', 'armDrive'},
	{'armDrive', 'driveready', 'armDriveready'},
	{'armDriveready', 'init', 'armInitWalk'},
}

-- E-stop handling
local allarm = {}
for i,v in ipairs(fsm.Arm) do allarm[v[1]] = true end
for k, v in pairs(allarm) do table.insert(fsm.Arm, {k, 'estop', 'armIdle'}) end





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



--------------------------------------------------------------------
---------------------- DRC Motion Code
--[[
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
--	{'motionStance', 'slowstep', 'motionSlowStep'},
	{'motionStance', 'stair', 'motionStepPreviewStair'},
	--
	{'motionHybridWalkInit', 'done', 'motionHybridWalk'},
	{'motionHybridWalk', 'done', 'motionHybridWalkEnd'},
	{'motionHybridWalk', 'emergency', 'motionStance'},
	{'motionHybridWalkEnd', 'done', 'motionStance'},
	--
	{'motionStepPreview', 'done', 'motionStance'},
	--
	{'motionStepPreviewStair', 'done', 'motionStance'},
	--
--	{'motionSlowStep', 'done', 'motionStance'},

	--DRIVE
	{'motionStance', 'driveready', 'motionDriveready'}, --untorque lower body
	{'motionDriveready', 'drive', 'motionDrive'}, --torque the body, enable foot control
	{'motionDrive', 'driveready', 'motionDriveready'}, --untorque lower body again
	{'motionDriveready', 'stand', 'motionInit'}, --torque all the legs and make the robot stand up


}
---------------------------------------------------------------------
]] --

--------------------------------------------------------------------
---------------------- Robocup Motion Code

fsm.Motion = {
	{'motionIdle', 'timeout', 'motionIdle'},
	{'motionIdle', 'stand', 'motionInit'},
	{'motionIdle', 'bias', 'motionBiasInit'},
	--
	{'motionBiasInit', 'done', 'motionBiasIdle'},
	{'motionBiasIdle', 'stand', 'motionInit'},
	--
	{'motionInit', 'done', 'motionStance'},
	{'motionUnInit', 'done', 'motionIdle'},
	--
	{'motionStance', 'bias', 'motionBiasInit'},
	{'motionStance', 'uninit', 'motionUnInit'},
	{'motionStance', 'hybridwalk', 'motionHybridWalkInit'},
  {'motionStance', 'done_step', 'motionHybridWalkKick'},
--	{'motionStance', 'slowstep', 'motionSlowStep'},

	--
	{'motionHybridWalkInit', 'done', 'motionHybridWalk'},
	{'motionHybridWalk', 'done', 'motionHybridWalkEnd'},
	{'motionHybridWalk', 'emergency', 'motionStance'},
	{'motionHybridWalkEnd', 'done', 'motionStance'},

	--kick handlings
  {'motionHybridWalk', 'done_step', 'motionHybridWalkKick'},
  {'motionHybridWalkKick', 'done', 'motionStance'},
  {'motionHybridWalkKick', 'walkalong', 'motionHybridWalk'},

	--
	{'motionStepPreview', 'done', 'motionStance'},
}

--------------------------------------------------------------------



-- E-stop handling
local allmotion = {}
for i,v in ipairs(fsm.Motion) do allmotion[v[1]] = true end
for k, v in pairs(allmotion) do table.insert(fsm.Motion, {k, 'estop', 'motionIdle'}) end


Config.fsm = fsm

return Config
