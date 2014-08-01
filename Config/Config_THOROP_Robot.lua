assert(Config, 'Need a pre-existing Config table!')
local vector = require'vector'

---------
-- IMU --
---------
Config.imu = {
  enabled = true,
  device = '/dev/ttyACM0',
  -- TODO: Add some mapping, etc.
}

Config.sensors = {
  head_lidar = false,
  chest_lidar = false,
  fsr = false,
  ft = false,
}

Config.right_ft = {
	unloaded = {
		1.5335531135531,
		1.6407326007326,
		1.3336996336996,
		1.0774358974359,
		1.6624908424908,
		1.8196336996337,
	},
  matrix = {
    {2.891145944, 1.952410267, 12.16354176, -353.8982028, -22.83263201, 332.4023596,},
    {-27.24742752,418.1117317,19.27221547,-203.7230186,-4.191511111,-193.4096575,},
    {555.0487287,33.30382696,565.428478,4.694309386,548.7347669,17.39183413,},
    {-0.781385227,5.906159073,-11.36879277,-3.070569682,12.10082004,-2.335516893,},
    {13.50667692,0.838353468,-7.268127443,4.929060231,-6.259577486,-4.959357264,},
    {0.590124577,-6.524081745,-0.01473672,-6.256832088,0.469615745,-5.887507744,},
    },
  gain = 1.556943243,
}

Config.left_ft = {
	unloaded = {
		1.2402197802198,
		1.5311355311355,
		1.3022710622711,
		1.4884249084249,
		1.1378754578755,
		1.3288644688645,
	},
  matrix = {
    {22.24512752, 3.52319111, 28.328385, -362.319417, -28.27411124, 363.7154308},
    {8.361978169, 406.1224392, 20.42296338, -206.7187762, 30.68868534, -209.0100171, },
    {562.2800788, 25.64039793, 558.264003, 44.90876394, 542.0300456, 28.76348109,},
    {0.018317069, 5.80239175, -11.37752081, -3.975201314, 12.53141658, -2.259373509,},
    {13.18015427, 0.655083305, -7.086161955, 4.577413702, -6.44142778, -5.53117315,},
    {-0.57439984, -6.312910806, 1.385922678, -6.259500022, 0.100189535, -6.528184117,},
  },
  gain = 1.613322338,
}

----------------
-- DCM Chains --
----------------
-- NOTE: Ignore the MX motors for now
Config.chain = {
	enabled = true
}
local right_arm = {
  name = 'rarm',
  ttyname = '/dev/ttyUSB0',
  --m_ids = {1,3,5,7,9,11,13, --[[head]] 29,30},
	enable_read = true, -- for the head...
}
local left_arm = {
  name = 'larm',
  ttyname = '/dev/ttyUSB1',
  --m_ids = {2,4,6,8,10,12,14,},
  --mx_ids = { 66,67,37, --[[lidar]] },
}
local right_leg = {
  name = 'rleg',
  ttyname = '/dev/ttyUSB2',
	-- waist pitch
  m_ids = {15,17,19, 21, 23,25, 28},
  enable_read = false,
}
local left_leg = {
  name = 'lleg',
  ttyname = '/dev/ttyUSB3',
	-- waist yaw
  m_ids = {16,18,20, 22, 24,26, 27},
  enable_read = false,
}
-- For RoboCup, use an MX only chain for the arms
local head_rc = {
  name = 'head',
  ttyname = '/dev/ttyUSB0',
  m_ids = {29, 30, 37, 11, 12, 13, 14},
  enable_read = true,
}
-- For RoboCup, use an MX only chain for the arms
local arms_rc = {
  name = 'arms',
  ttyname = '/dev/ttyUSB1',
  m_ids = {11, 12, 13, 14},
  enable_read = true,
}

if OPERATING_SYSTEM=='darwin' then
  right_arm.device = '/dev/cu.usbserial-FTVTLUY0A'
  left_arm.device  = '/dev/cu.usbserial-FTVTLUY0B'
  right_leg.device = '/dev/cu.usbserial-FTVTLUY0C'
  left_leg.device  = '/dev/cu.usbserial-FTVTLUY0D'
end
if ONE_CHAIN then
  -- Add the one chain support
  local one_chain = {
    device = '/dev/ttyUSB0',
  }
	if OPERATING_SYSTEM=='darwin' then
		one_chain.device = '/dev/cu.usbserial-FTVTLUY0A'
	end
  for _,v in ipairs(right_arm.m_ids) do table.insert(one_chain.m_ids, v) end
  for _,v in ipairs(left_arm.m_ids)  do table.insert(one_chain.m_ids, v) end
  for _,v in ipairs(right_leg.m_ids) do table.insert(one_chain.m_ids, v) end
  for _,v in ipairs(left_leg.m_ids)  do table.insert(one_chain.m_ids, v) end
  table.insert(Config.chain, one_chain)
  right_arm = nil
  left_arm  = nil
  right_leg = nil
  left_leg  = nil
else
  table.insert(Config.chain, right_leg)
  table.insert(Config.chain, left_leg)
  if Config.USE_DUMMY_ARMS then
    -- Not set up yet...
	  --table.insert(Config.chain, arms_rc)
  	table.insert(Config.chain, head_rc)
  else
    table.insert(Config.chain, right_arm)
    table.insert(Config.chain, left_arm)
  end
  one_chain = nil
end

-- Shared memory layout for default THOR-OP
local indexHead = 1   -- Head: 1 2
local nJointHead = 2
local indexLArm = 3   --LArm: 3 4 5 6 7 8 9
local nJointLArm = 7
local indexLLeg = 10  --LLeg: 10 11 12 13 14 15
local nJointLLeg = 6
local indexRLeg = 16  --RLeg: 16 17 18 19 20 21
local nJointRLeg = 6
local indexRArm = 22  --RArm: 22 23 24 25 26 27 28
local nJointRArm = 7
local indexWaist = 29  --Waist: 29 30
local nJointWaist = 2
local indexLGrip = 31 -- Grippers
local nJointLGrip = 2
local indexRGrip = 33
local nJointRGrip = 2
local indexLidar = 35 -- Lidar
local nJointLidar = 1
local nJoint = 35

Config.parts = {
	Head=vector.count(indexHead,nJointHead),
	LArm=vector.count(indexLArm,nJointLArm),
	LLeg=vector.count(indexLLeg,nJointLLeg),
	RLeg=vector.count(indexRLeg,nJointRLeg),
	RArm=vector.count(indexRArm,nJointRArm),
	Waist=vector.count(indexWaist,nJointWaist),
	LGrip=vector.count(indexLGrip,nJointLGrip),
  RGrip=vector.count(indexRGrip,nJointRGrip),
  Lidar=vector.count(indexLidar,nJointLidar)
}

----------------------
-- Servo Properties --
----------------------
local servo = {}
servo.joint_to_motor={
  29,30,  --Head yaw/pitch
  2,4,6,8,10,12,14, --LArm
  16,18,20,22,24,26, -- left leg
  15,17,19,21,23,25, -- right leg
  1,3,5,7,9,11,13,  --RArm
  27,28, --Waist yaw/pitch
  66,67, -- left gripper/trigger
  70,65, -- right gripper/trigger
  37, -- Lidar pan
}

--http://support.robotis.com/en/product/dynamixel_pro/control_table.htm#Actuator_Address_611
-- TODO: Use some loop based upon MX/NX
-- TODO: some pros are different
servo.steps = 2 * vector.new({
  151875,151875, -- Head
  251000,251000,251000,251000,151875,151875,151875, --LArm
  251000,251000,251000,251000,251000,251000, --LLeg
  251000,251000,251000,251000,251000,251000, --RLeg
  251000,251000,251000,251000,151875,151875,151875, --RArm
  251000,251000, -- Waist
  2048,2048, -- Left gripper
  2048,2048, -- Right gripper
  2048, -- Lidar pan
})

-- NOTE: Servo direction is webots/real robot specific
servo.direction = vector.new({
  1,1, -- Head
  1,-1,1,1,1,1,1, --LArm
  ------
  -1, -1,1,   1,  -1,1, --LLeg
  -1, -1,-1, -1,  1,1, --RLeg
  ------
  -1,-1,1,-1, 1,1,1, --RArm
  1,1, -- Waist
  1,1, -- left gripper TODO
  1,1, -- right gripper TODO
  -1, -- Lidar pan
})

-- TODO: Offset in addition to bias?
servo.rad_offset = vector.new({
  0,0, -- Head
  -90,90,-90,45,90,0,0, --LArm
  0,0,0,-45,0,0, --LLeg
  0,0,0,45,0,0, --RLeg
  90,-90,90,-45,-90,0,0, --RArm
  0,0, -- Waist
  0,0, -- left gripper
  0,0, -- right gripper
  0, -- Lidar pan
})*DEG_TO_RAD

--SJ: Arm servos should at least move up to 90 deg
servo.min_rad = vector.new({
  ---90,-80, -- Head
  -135,-80, -- Head
  -90, 0, -90,    -160,   -180,-87,-180, --LArm
  -175,-175,-175,-175,-175,-175, --LLeg
  -175,-175,-175,-175,-175,-175, --RLeg
  -90,-87,-90,    -160,   -180,-87,-180, --RArm
  -90,-45, -- Waist
  0, -90,
  0, -90,
  -60, -- Lidar pan
})*DEG_TO_RAD

servo.max_rad = vector.new({
  --90,80, -- Head
  135,80, -- Head
  160,87,90,   0,     180,87,180, --LArm
  175,175,175,175,175,175, --LLeg
  175,175,175,175,175,175, --RLeg
  160,-0,90,   0,     180,87,180, --RArm
  90,45, -- Waist
  90,20,
  90,20,
  60, -- Lidar pan
})*DEG_TO_RAD

if Config.USE_DUMMY_ARMS then
	indexHead = 1   -- Head: 1 2
	nJointHead = 2
	indexLArm = 3   --LArm: 3 4
	nJointLArm = 2
	indexLLeg = 5  --LLeg:  5 6 7 8 9 10
	nJointLLeg = 6
	indexRLeg = 11  --RLeg: 16 17 18 19 20 21
	nJointRLeg = 6
	indexRArm = 17  --RArm: 22 23 24 25 26 27 28
	nJointRArm = 2
	indexWaist = 19  --Waist: 29 30
	nJointWaist = 2
	indexLidar = 21
	nJointLidar = 1
	nJoint = 21
	--
	Config.parts = {
		Head=vector.count(indexHead,nJointHead),
		LArm=vector.count(indexLArm,nJointLArm),
		LLeg=vector.count(indexLLeg,nJointLLeg),
		RLeg=vector.count(indexRLeg,nJointRLeg),
		RArm=vector.count(indexRArm,nJointRArm),
		Waist=vector.count(indexWaist,nJointWaist),
		Lidar=vector.count(indexLidar,nJointLidar)
	}
	--
	servo.joint_to_motor={
		29,30,  --Head yaw/pitch
		11,13,  --LArm
		16,18,20,22,24,26, -- left leg
		15,17,19,21,23,25, -- right leg
		12,14, --RArm
		27,28, --Waist yaw/pitch
		37, -- Lidar pan
	}
	--
	servo.steps = 2 * vector.new({
		151875, 151875, -- Head
		151875, 151875, --LArm
		251000,251000,251000,251000,251000,251000, --LLeg
		251000,251000,251000,251000,251000,251000, --RLeg
		151875, 151875, --RArm
		251000, 251000, -- Waist
		2048, -- Lidar pan
	})
	--
	servo.direction = vector.new({
		1,1, -- Head
		-1,1, --LArm
		------
		-1, -1,1,   1,  -1,1, --LLeg
		-1, -1,-1, -1,  1,1, --RLeg
		------
		-1, -1, --RArm
		1,1, -- Waist
		-1, -- Lidar pan
	})
	--
	servo.rad_offset = vector.new({
		0,0, -- Head
		0, -90, --LArm
		0,0,0,-45,0,0, --LLeg
		0,0,0,45,0,0, --RLeg
		0, 90, --RArm
		0,0, -- Waist
		0, -- Lidar pan
	})*DEG_TO_RAD

	--SJ: Arm servos should at least move up to 90 deg
	servo.min_rad = vector.new({
		---90,-80, -- Head
		-135,-80, -- Head
		-90, -90, --LArm
		-175,-175,-175,-175,-175,-175, --LLeg
		-175,-175,-175,-175,-175,-175, --RLeg
		-90,-90, --RArm
		-90,-45, -- Waist
		-60, -- Lidar pan
	})*DEG_TO_RAD

	servo.max_rad = vector.new({
		--90,80, -- Head
		135,80, -- Head
		90, 135, --LArm
		175,175,175,175,175,175, --LLeg
		175,175,175,175,175,175, --RLeg
		90, 135, --RArm
		90,45, -- Waist
		60, -- Lidar pan
	})*DEG_TO_RAD
end

-- If the motor can rotate in extended mode
local is_unclamped = {}
for idx=1,#servo.min_rad do
  if servo.max_rad[idx] == 180*DEG_TO_RAD and servo.min_rad[idx]==-180*DEG_TO_RAD then
    is_unclamped[idx] = true
  else
    is_unclamped[idx] = false
  end
end
servo.is_unclamped = is_unclamped

-- Convienence tables to go between steps and radians
servo.moveRange = 360 * DEG_TO_RAD * vector.ones(nJoint)
-- Step<-->Radian ratios
servo.to_radians = vector.zeros(nJoint)
servo.to_steps = vector.zeros(nJoint)
for i,nsteps in ipairs(servo.steps) do
  servo.to_steps[i] = nsteps / servo.moveRange[i]
  servo.to_radians[i] = servo.moveRange[i] / nsteps
end
-- Set the step zero
-- NOTE: rad zero is always zero
servo.step_zero = vector.new(servo.steps) / 2
for i,nsteps in ipairs(servo.steps) do
  -- MX is halfway, while NX is 0 and goes positive/negative
  if nsteps~=4096 then servo.step_zero[i] = 0 end
end
servo.step_offset = vector.zeros(nJoint)
for i, offset in ipairs(servo.rad_offset) do
  servo.step_offset[i] = offset * servo.to_steps[i]
end

if IS_WEBOTS then
  -- Webots overrides tested in Webots 7.2.3, with ShortNewHand
  servo.direction = vector.new({
    -1,-1, -- Head
    1,-1,-1,  1,  -1,-1,-1, --LArm
    --[[Yaw/Roll:]] -1, 1, --[[3 Pitch:]] 1,1, 1, 1, --LLeg
    --[[Yaw/Roll:]] -1, 1, --[[3 Pitch:]] 1,1, 1, 1, --RLeg
    1,-1,-1,  -1,  -1,-1,-1, --RArm
    -- TODO: Check the gripper
    -1,1, -- Waist
    1,-1, -- left gripper
    -1,-1, -- right gripper

    1, -- Lidar pan
  })

  servo.rad_offset = vector.new({
--    0,0, -- head
-- I don't know why but the webots model has stupid 10 degree offset...
-- And this killed all our webots vision for months!

    0,-10 ,
    -90,0,0,  0,  0,0,0,
    0,0,0,0,0,0,
    0,0,0,0,0,0,
    -180,0,0,  0,  0,0,0,
    0,0,
    0,0,
    0,0,
    60,
  })*DEG_TO_RAD

  servo.min_rad = vector.new({
		-135,-80, -- Head
		-90, 0, -90, -160,      -180,-87,-180, --LArm
		-175,-175,-175,-175,-175,-175, --LLeg
		-175,-175,-175,-175,-175,-175, --RLeg
		-90,-180,-90,-160,       -180,-87,-180, --RArm
		-90,-45, -- Waist
		120,80, --lhand
		120,60,--rhand

		-60, -- Lidar pan
	})*DEG_TO_RAD

	servo.max_rad = vector.new({
		135, 80, -- Head
		160,180,90,0,     180,87,180, --LArm
		175,175,175,175,175,175, --LLeg
		175,175,175,175,175,175, --RLeg
		160,-0,90,0,     180,87,180, --RArm
		90,79, -- Waist
		0,45,  --lhand
		0,45,    --rhand
		60, -- Lidar pan
	})*DEG_TO_RAD

end

assert(#servo.rad_offset==nJoint,'Bad servo rad_offset!')
assert(#servo.min_rad==nJoint,'Bad servo min_rad!')
assert(#servo.max_rad==nJoint,'Bad servo max_rad!')
assert(#servo.steps==nJoint,'Bad servo steps!')
assert(#servo.direction==nJoint,'Bad servo direction!')
assert(#servo.joint_to_motor==nJoint,'Bad servo id map!')
-- Make the reverse map
servo.motor_to_joint = {}
for j,m in ipairs(servo.joint_to_motor) do servo.motor_to_joint[m] = j end

-- Export
Config.servo = servo
