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

Config.right_ft = {   --
	id = 'FT13288 TWE',
-- Robotis: 1.372045898 1.824023438 1.604882813 2.045581055 1.819995117 1.914257813
  unloaded= {1.385, 1.860, 1.636, 2.085, 1.855, 1.940},
  matrix = {
{4.900247363, 3.309169943, 20.61617248, -599.8274623, -38.69937629, 563.3938299},
{-46.18208054, 708.6639521, 32.66477198, -345.2932518, -7.10425612, -327.8129788},
{940.7605571, 56.44716433, 958.3533526, 7.956456586, 930.0589269, 29.47768497},
{-1.32438174, 10.01043911, -19.26914029, -5.204355394, 20.50986448, -3.958503209},
{22.89267275, 1.420938081, -12.31886007, 8.354339375, -10.60945337, -8.405690278},
{1.000211147, -11.05776567-0.02497749085, -10.60480015, 0.7959588898, -9.978826685}
},
  gain = 1,
}

Config.left_ft = {
	id = 'FT14216 TWE',
	--Robotis: 1.752319336	1.839331055	1.696728516	1.773266602	1.718481445	1.874780273
	unloaded = {1.765, 1.843, 1.705, 1.792, 1.744, 1.880},
	matrix = {
		{-17.58726087, 2.862495136, 72.47124518, -572.1905501, -137.501991, 579.6660841},
		{-43.6278751, 680.2963926, 36.9166943, -321.8118551, 76.20995953, -339.6674833},
		{909.0217452, 40.32971767, 966.5665832, 36.56958695, 903.3278882, 55.75072623},
		{-0.6061349454, 9.760197509, -19.52983969, -5.367851785, 20.50381105, -3.712027},
		{22.95412835, 1.04231606, -12.37184692, 7.756497574, -9.834291952, -9.06925662},
		{0.7797247308, -10.36390078, 0.6156105021, -9.954750945, 2.629862112, -10.27711385},
	},
  gain = 1,
}
Config.left_ft.m_ids = {24,26}--{26, 24}
Config.right_ft.m_ids = {25, 23}

-- DCM Chains
Config.chain = {
	enabled = true
}
local right_arm = {
  name = 'rarm',
  ttyname = '/dev/ttyUSB0',
  m_ids = {1,3,5,7,9,11,13,
        --head
        29, 30,
				-- gripper
--				66, 67
        },
	enable_read = true,
}
local left_arm = {
  name = 'larm',
  ttyname = '/dev/ttyUSB1',
  m_ids = {2,4,6,8,10,12,14,
  -- lidar
  37,
	-- gripper
  66, 67
},
  enable_read = true
}
local right_leg = {
  name = 'rleg',
  ttyname = '/dev/ttyUSB2',
	-- waist pitch
  m_ids = {15,17,19, 21, 23,25, 28},
  enable_read = true,
}
local left_leg = {
  name = 'lleg',
  ttyname = '/dev/ttyUSB3',
	-- waist yaw
  m_ids = {16,18,20, 22, 24,26, 27},
  enable_read = true,
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
  right_arm.ttyname = '/dev/cu.usbserial-FTVTLUY0A'
  left_arm.ttyname = '/dev/cu.usbserial-FTVTLUY0B'
  right_leg.ttyname = '/dev/cu.usbserial-FTVTLUY0C'
  left_leg.ttyname  = '/dev/cu.usbserial-FTVTLUY0D'
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
	-- Both keys and indices
	Config.chain[right_leg.name] = right_leg
	Config.chain[left_leg.name] = left_leg
  table.insert(Config.chain, right_leg)
  table.insert(Config.chain, left_leg)
  if Config.USE_DUMMY_ARMS then
    -- Not set up yet...
	  --table.insert(Config.chain, arms_rc)
  	table.insert(Config.chain, head_rc)
  else
    table.insert(Config.chain, right_arm)
    table.insert(Config.chain, left_arm)
		Config.chain[right_arm.name] = right_arm
		Config.chain[left_arm.name] = left_arm
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
	Head = vector.count(indexHead,nJointHead),
	LArm = vector.count(indexLArm,nJointLArm),
	LLeg = vector.count(indexLLeg,nJointLLeg),
	RLeg = vector.count(indexRLeg,nJointRLeg),
	RArm = vector.count(indexRArm,nJointRArm),
	Waist = vector.count(indexWaist,nJointWaist),
	LGrip = vector.count(indexLGrip,nJointLGrip),
  RGrip = vector.count(indexRGrip,nJointRGrip),
  Lidar = vector.count(indexLidar,nJointLidar)
}

local jointNames = {
  "Neck","Head", -- Head (Yaw,pitch)
  -- Left Arm
  "ShoulderL", "ArmUpperL", "LeftShoulderYaw",
  "ArmLowerL","LeftWristYaw","LeftWristRoll","LeftWristYaw2",
  -- Left leg
  "PelvYL","PelvL","LegUpperL","LegLowerL","AnkleL","FootL",
  -- Right leg
  "PelvYR","PelvR","LegUpperR","LegLowerR","AnkleR","FootR",
  --Right arm
  "ShoulderR", "ArmUpperR", "RightShoulderYaw","ArmLowerR",
  "RightWristYaw","RightWristRoll","RightWristYaw2",
  -- Waist
  "TorsoYaw","TorsoPitch",
  -- Gripper
  "l_grip", "l_trigger",
  "r_grip", "r_trigger",
  -- lidar movement
  "ChestLidarPan",
}
Config.jointNames = jointNames

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
  --70,65, -- left gripper/trigger
  --66,67, -- right gripper/trigger
  -- Swapped gripper
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
  1,-1, -- Head
  1,-1,1,1,1,1,1, --LArm
  ------
  -1, -1,1,   1,  -1,1, --LLeg
  -1, -1,-1, -1,  1,1, --RLeg
  ------
  -1,-1,1,-1, 1,1,1, --RArm
  -1, -1, -- Waist
  1,-1, -- left gripper TODO
  -1,1, -- right gripper (Verified 9/8/2014)
  -1, -- Lidar pan
})

-- TODO: Offset in addition to bias?
servo.rad_offset = vector.new({
  0,140.41837037037, -- Head
  -90,90,-90,45,90,0,0, --LArm
  0,0,0,-45,0,0, --LLeg
  0,0,0,45,0,0, --RLeg
  90,-90,90,-45,-90,0,0, --RArm
  0,0, -- Waist
  75,-80, -- left gripper
  35, -35, -- right gripper
  0, -- Lidar pan
})*DEG_TO_RAD

--SJ: Arm servos should at least move up to 90 deg
servo.min_rad = vector.new({
  ---90,-80, -- Head
  -135,-80, -- Head
  -90, 0, -90,    -160,   -180,-87,-180, --LArm
  -175,-25,-175,-175,-175,-175, --LLeg
  -175,-175,-175,-175,-175,-175, --RLeg
  -90,-87,-90,    -160,   -180,-87,-180, --RArm
  -90,-45, -- Waist
  -30, -30,
  -90, -90,
  -60, -- Lidar pan
})*DEG_TO_RAD

servo.max_rad = vector.new({
  --90,80, -- Head
  135,80, -- Head
  160,87,90,   0,     180,87,180, --LArm
  175,25,175,175,175,175, --LLeg
  175,175,175,175,175,175, --RLeg
  160,-0,90,   0,     180,87,180, --RArm
  90,45, -- Waist
  65,65, -- lgrip
  90,90,
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
		1,-1, -- Head
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
	print('DUMMY ARMS RAD OFFSET')
	servo.rad_offset = vector.new({
		-180,135, -- Head
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
   	1,1, -- Head
    1,-1,-1,  1,  -1,-1,1, --LArm
    --[[Yaw/Roll:]] 1, 1, --[[3 Pitch:]] 1,1, 1, 1, --LLeg
    --[[Yaw/Roll:]] 1, 1, --[[3 Pitch:]] 1,1, 1, 1, --RLeg
    1,-1,-1,  -1,  -1,-1,1, --RArm
    -- TODO: Check the gripper
    -1,1, -- Waist
    1,-1, -- left gripper
    1,1, -- right gripper

    1, -- Lidar pan
  })

  servo.rad_offset = vector.new({
    0,0, -- head
    -90,0,0,  0,  0,0,0,
    0,0,0,0,0,0,
    0,0,0,0,0,0,
    -90,0,0,  0,  0,0,0,
    0,0,
    0,0,
    0,0,
    0,
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

assert(#jointNames==nJoint,'Bad servo rad_offset!')
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
