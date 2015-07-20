assert(Config, 'Need a pre-existing Config table!')
local vector = require'vector'

Config.nJoint = 37

---------
-- IMU --
---------
Config.imu = {
	enabled = true,
	device = '/dev/ttyACM0',
	-- TODO: Add some mapping, etc.
}

local FT13288 = {   --
id = 'FT13288 TWE',
-- Robotis: 1.372045898 1.824023438 1.604882813 2.045581055 1.819995117 1.914257813
-- Original before changing voltage for Chip: unloaded= {1.385, 1.860, 1.636, 2.085, 1.855, 1.940},
unloaded = {1.71687, 1.93279, 1.8772, 2.01174, 1.91473, 1.97597}, -- For Dale
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

local FT14216 = {
	id = 'FT14216 TWE',
	-- Robotis: 1.752319336	1.839331055	1.696728516	1.773266602	1.718481445	1.874780273
	-- Original before changing voltage for Chip: unloaded = {1.765, 1.843, 1.705, 1.792, 1.744, 1.880},
	unloaded = {1.64597, 1.6782, 1.5082, 1.6645, 1.54806, 1.69213}, -- For Dale
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
local FT14217 = {
	id = 'FT14217 TWE',
	-- Robotis: 1.752319336	1.839331055	1.696728516	1.773266602	1.718481445	1.874780273
	-- Original before changing voltage for Chip: unloaded = {1.765, 1.843, 1.705, 1.792, 1.744, 1.880},
	unloaded = {1.64597, 1.6782, 1.5082, 1.6645, 1.54806, 1.69213}, -- For Dale
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

local FT16464 = {
	id = 'FT16464 TWE',
	--Robotis: 1.876391602	1.590380859	1.536401367	1.54284668	1.399438477	1.550097656
	unloaded = {1.876391602, 1.590380859, 1.536401367, 1.54284668, 1.399438477, 1.550097656},
	matrix = {
		{-5.929833893, 0.8325384264, 5.46689167, -212.1689223, -6.476323927, 214.0013344, },
		{-8.690056049, 245.2349123, 2.607437873, -121.945352, 6.568826403, -123.8082159, },
		{332.57748, 14.81662867, 331.0787239, 21.31009399, 328.0365411, 17.59399945, },
		{-0.1150545763, 1.689229156, -5.258416984, -1.218932387, 5.37573244, -0.5302761172, },
		{6.224272472, 0.2887898915, -3.153530427, 1.246742901, -2.997598677, -1.654413777, },
		{0.1098161854, -3.139490306, 0.03337703537, -3.12974112, 0.2336113953, -3.161907375, },
	},
	gain = 1,
}

local FT16465 = {
	id = 'FT16465 TWE',
	--Robotis: 1.634692383	1.774072266	2.123730469	1.978710938	1.681420898	1.795019531
	unloaded = {1.634692383, 1.774072266, 2.123730469, 1.978710938, 1.681420898, 1.795019531},
	matrix = {
		{3.918484874, 1.607657431, 10.12798524, -221.1586464, -24.15288317, 225.1144886,},
		{-14.5668872, 259.7992641, 6.51641097, -127.5443307, 9.111272978, -130.5197947,},
		{330.4094409, 21.65213152, 327.5531325, 18.19960965, 331.4757537, 20.76524139,},
		{-0.1191281486, 1.815650512, -5.208627943, -1.223158945, 5.392642987, -0.5363023538,},
		{6.106923853, 0.4093856657, -3.157167318, 1.336322243, -2.876780379, -1.758075016,},
		{0.2809996275, -3.333765233, 0.1823969378, -3.259335136, 0.2372790236, -3.360978917,},
	},
	gain = 1,
}

local FT16389 = {
	id = 'FT16389 TWE',
	--Robotis: 1.953735352	2.201074219	2.052026367	2.441967773	1.695117188	2.275195313
	unloaded = {1.971, 2.2, 2.077, 2.44, 1.712, 2.2283},
	matrix = {
		{22.49073348, 0.3025440001, 36.26925449, -547.3221321, -21.50321143, 541.75967},
		{-27.91065023, 638.4107838, 19.65818873, -313.8963823, -9.077283315, -316.5070849},
		{907.4192168, -0.4230087475, 905.9567343, -6.396575699, 883.8117013, -17.31417173},
		{-0.7172646123, 8.992764085, -19.26613378, -4.307392273, 19.64181521, -4.703897346},
		{22.14885554, -0.04851174637, -11.72558508, 7.769243185, -10.84145254, -7.499068369},
		{0.8030554316, -9.406177096, 0.6551973419, -9.171127658, -0.3493049293, -9.085148473},
	},
	gain = 1,
}

local FT16390 = {
	id = 'FT16390 TWE',
	--Robotis: 1.634692383	2.378320313	1.600854492	1.700756836	1.520288086	1.979516602
	unloaded = {1.64, 2.34, 1.6, 1.67, 1.55, 1.98},
	matrix = {
{2.794322236, -0.9222704694, 9.966766489, -551.0586614, 10.42278336, 540.9036387},
{-0.4292623116, 633.7833909, -2.281245747, -317.6652813, -7.196761641, -313.5569844},
{892.9603907, -51.7800059, 899.4666346, -36.15147671, 890.2589678, -6.153909368},
{-1.397285, 9.01001264, -18.60247505, -3.732307941, 19.96151081, -4.608539693},
{22.31925586, -1.259330382, -12.49663622, 8.357565453, -10.32328044, -7.486074628},
{0.1269012243, -9.334903262, 0.1334557736, -9.390952799, -0.3282695074, -9.150568141},
	},
	gain = 1,
}

Config.right_foot_ft = FT16390
Config.left_foot_ft = FT16389

Config.right_wrist_ft = FT16465
Config.left_wrist_ft = FT16464


--temporary testing for dale's FT sensors (from penn)
---------------------------------
---------------------------------
---------------------------------
--Config.right_foot_ft = FT13288
--Config.left_foot_ft = FT14216
---------------------------------
---------------------------------
---------------------------------


--FT BIAS
Config.left_foot_ft.bias_forceZ = 49
Config.left_foot_ft.bias_torque = {-6,13.8}

Config.right_foot_ft.bias_forceZ = -20
Config.right_foot_ft.bias_torque = {-10,18.5}

Config.left_foot_ft = FT16389
Config.right_foot_ft = FT16390
Config.left_foot_ft.m_ids = {24, 26}
Config.right_foot_ft.m_ids = {23, 25}







-- DCM Chains
Config.chain = {enabled = true}

local right_arm = {
	name = 'rarm',
	ttyname = '/dev/ttyUSB0',
	m_ids = {
--	1,3,5,7,9,11,13,

	-- Stub arms
	1,2,

	-- waist
	27, 28,
	--head
	29, 30,
	-- gripper
--	63, 65, 67
	},
	enable_read = true,
}

local left_arm = {
	name = 'larm',
	ttyname = '/dev/ttyUSB1',
	m_ids = {
--	2,4,6,8,10,12,14,
	-- lidar
--	37,
	--no gripper
	},
	enable_read = true
}

local right_leg = {
	name = 'rleg',
	ttyname = '/dev/ttyUSB2',
	-- waist pitch
--	m_ids = {15,17,19, 21, 23,25, 27},
	m_ids = {15,17,19, 21, 23,25},
	enable_read = true,
}

local left_leg = {
	name = 'lleg',
	ttyname = '/dev/ttyUSB3',
	-- waist yaw
	m_ids = {16,18,20, 22, 24,26},
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
----[[
		table.insert(Config.chain, right_arm)
		table.insert(Config.chain, left_arm)
		Config.chain[right_arm.name] = right_arm
		Config.chain[left_arm.name] = left_arm
--]]
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
local nJointLGrip = 3
local indexRGrip = 34
local nJointRGrip = 3
local indexLidar = 37 -- Lidar
local nJointLidar = 1
local nJoint = 37

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

print("HEREHERE")
local jointNames = {
	"Neck","Head", -- Head (Yaw,pitch)
	-- Left Arm
--	"ShoulderL", "ArmUpperL", "LeftShoulderYaw",

	"ShoulderL", nil,nil,

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
	"l_grip", "l_trigger", "l_extra",
	"r_grip", "r_trigger", "r_extra",
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
	28,27, --Waist yaw/pitch (mk2 is inverted)
	64,66,68, -- left gripper/trigger (This is the order)
	63,67,65, -- right gripper/trigger/extra
	37, -- Lidar pan
}

--http://support.robotis.com/en/product/dynamixel_pro/control_table.htm#Actuator_Address_611
-- TODO: Use some loop based upon MX/NX
-- TODO: some pros are different
servo.steps = 2 * vector.new({
	151875,151875, -- Head
--	251000,251000,251000,251000,151875,151875,151875, --LArm (mk1 arm)
--	251000,251000,251000,251000,251000,151875,151875, --LArm (mk2 arm)
	151875,251000,251000,251000,251000,151875,151875, --LArm (RC arm)
	251000,251000,251000,251000,251000,251000, --LLeg
	251000,251000,251000,251000,251000,251000, --RLeg
--	251000,251000,251000,251000,151875,151875,151875, --RArm
--	251000,251000,251000,251000,251000,151875,151875, --RArm
	151875,251000,251000,251000,251000,151875,151875, --RArm (RC arm)
	251000,251000, -- Waist
	2048,2048,2048, -- Left gripper
	2048,2048,2048, -- Right gripper
	2048, -- Lidar pan
})



-- NOTE: Servo direction is webots/real robot specific
servo.direction = vector.new({
	1,1, -- Head, mk2

	1,1,1, 1, 1,1,1, --LArm, mk2 reassembled
	------
	-1, 1,1,   1,  1,1, --LLeg
	-1, 1,-1, -1,  -1,1, --RLeg
	------
	-1,1,1, -1, 1,1,1, --RArm, mk2 reassembled
	1, 1, -- Waist, mk2
	-1,1,-1, -- left gripper TODO
	1,1,-1, -- right gripper/trigger (Good trigger with UCLA hand)
	-1, -- Lidar pan
})

-- TODO: Offset in addition to bias?
servo.rad_offset = vector.new({
	0,0, -- Head
--	-90,  -90,  -90,45,  -90,0,0, --LArm
	0,  -90,  -90,45,  -90,0,0, --LArm, RC
	0,0,0,  0  ,0,0, --LLeg
	0,0,0,  0  ,0,0, --RLeg
--	90,  90,  90,-45,  90,0,0, --RArm
	0,  90,  90,-45,  90,0,0, --RArm, RC
--	90,  90,  90,-45,  -90,0,0, --RArm
	0,0, -- Waist
	0, 0, 0, -- left gripper/trigger
	-90, -90, 0, -- right gripper/trigger (UCLA verified)
	0, -- Lidar pan
})*DEG_TO_RAD

--SJ: Arm servos should at least move up to 90 deg
servo.min_rad = vector.new({
	-135,-80, -- Head
	-90, 0, -90,    -160,   -180,-87,-180, --LArm
	-175,-25,-175,-175,-175,-175, --LLeg
	-175,-175,-175,-175,-175,-175, --RLeg
	-90,-87,-90,    -160,   -180,-87,-180, --RArm
--	-90,-45, -- Waist
	-90,-45, -- Waist
	-60, -55, -60,
	-90, -120, -55, -- right gripper/trigger (UCLA verified)
	-60, -- Lidar pan
})*DEG_TO_RAD

servo.max_rad = vector.new({
	--90,80, -- Head
	135,80, -- Head
--	160,87,90,   0,     180,87,180, --LArm
--FOR DRIVING

	180,87,135,   0,     180,87,180, --LArm

	175,25,175,175,175,175, --LLeg
	175,175,175,175,175,175, --RLeg
--	160,-0,90,   0,     180,87,180, --RArm
	160,-0,90,   0,     180,87,180, --RArm
--	90,45, -- Waist
	90,45, -- Waist
	65,65,55, -- lgrip
	105,110,105, -- right gripper/trigger (UCLA verified)
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
		1,1,1, -- left gripper
		-1,-1,-1, -- right gripper

		1, -- Lidar pan
	})

	servo.rad_offset = vector.new({
		0,0, -- head
		-90,0,0,  0,  0,0,0,
		0,0,0,0,0,0,
		0,0,0,0,0,0,
		-90,0,0,  0,  0,0,0,
		0,0,
		0,0,0,
		0,0,0,
		0,
	})*DEG_TO_RAD

	servo.min_rad = vector.new({
		-135,-80, -- Head
		-90, 0, -90, -160,      -180,-87,-180, --LArm
		-175,-175,-175,-175,-175,-175, --LLeg
		-175,-175,-175,-175,-175,-175, --RLeg
		-90,-180,-90,-160,       -180,-87,-180, --RArm
--		-90,-45, -- Waist
		-30,0, -- Waist
		120,80,60, --lhand
		120,60,60,--rhand

		-60, -- Lidar pan
	})*DEG_TO_RAD

	servo.max_rad = vector.new({
		135, 80, -- Head
		160,180,90,0,     180,87,180, --LArm
		175,175,175,175,175,175, --LLeg
		175,175,175,175,175,175, --RLeg
		160,0,90,0,     180,87,180, --RArm
--		90,79, -- Waist
		179,79, -- Waist
		0,45,45,  --lhand
		0,45,45,    --rhand
		60, -- Lidar pan
	})*DEG_TO_RAD


end



--[[
print("Robot config loading")
print(#jointNames,
			#servo.rad_offset, --
			#servo.min_rad,    --
			#servo.steps,
			#servo.direction,  --
			#servo.joint_to_motor)
print(nJoint)
--]]

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
