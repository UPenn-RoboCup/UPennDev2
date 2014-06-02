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

----------------
-- DCM Chains --
----------------
-- NOTE: Ignore the MX motors for now
Config.chain = {
	enabled = true
}
local right_arm = {
  name = 'rarm',
  device = '/dev/ttyUSB0',
  m_ids = {1,3,5,7,9,11,13, --[[head]] 29,30},
  --mx_ids = { 70,65 },
}
local left_arm = {
  name = 'larm',
  device = '/dev/ttyUSB1',
  m_ids = {2,4,6,8,10,12,14,},
  --mx_ids = { 66,67,37, --[[lidar]] },
}
local right_leg = {
  name = 'rleg',
  device = '/dev/ttyUSB2',
  m_ids = {15,17,19,21,23,25, --[[waist pitch]]28},
}
local left_leg = {
  name = 'lleg',
  device = '/dev/ttyUSB3',
  m_ids = {16,18,20,22,24,26, --[[waist yaw]]27}
}
-- Add the one chain support
local one_chain = {
  device = '/dev/ttyUSB0',
  m_ids = {},
}
for _,v in ipairs(right_arm.m_ids) do table.insert(one_chain.m_ids, v) end
for _,v in ipairs(left_arm.m_ids)  do table.insert(one_chain.m_ids, v) end
for _,v in ipairs(right_leg.m_ids) do table.insert(one_chain.m_ids, v) end
for _,v in ipairs(left_leg.m_ids)  do table.insert(one_chain.m_ids, v) end

if OPERATING_SYSTEM=='darwin' then
  right_arm.device = '/dev/cu.usbserial-FTVTLUY0A'
  left_arm.device  = '/dev/cu.usbserial-FTVTLUY0B'
  right_leg.device = '/dev/cu.usbserial-FTVTLUY0C'
  left_leg.device  = '/dev/cu.usbserial-FTVTLUY0D'
  one_chain.device = '/dev/cu.usbserial-FTVTLUY0A'
end
if ONE_CHAIN then
  table.insert(Config.chain, one_chain)
  right_arm = nil
  left_arm  = nil
  right_leg = nil
  left_leg  = nil
else
  --table.insert(Config.chain, right_arm)
  --table.insert(Config.chain, left_arm)
  table.insert(Config.chain, right_leg)
  table.insert(Config.chain, left_leg)
  one_chain = nil
end

----------------------
-- Servo Properties --
----------------------
local nJoint = 35
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
assert(#servo.joint_to_motor==nJoint,'Bad servo id map!')
-- Make the reverse map
servo.motor_to_joint = {}
for j,m in ipairs(servo.joint_to_motor) do servo.motor_to_joint[m] = j end

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
assert(#servo.steps==nJoint,'Bad servo steps!')

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
assert(#servo.direction==nJoint,'Bad servo direction!')

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
assert(#servo.rad_offset==nJoint,'Bad servo rad_offset!')

--SJ: Arm servos should at least move up to 90 deg
servo.min_rad = vector.new({
  -90,-80, -- Head
  -90, 0, -90,    -160,   -180,-87,-180, --LArm
  -175,-175,-175,-175,-175,-175, --LLeg
  -175,-175,-175,-175,-175,-175, --RLeg
  -90,-87,-90,    -160,   -180,-87,-180, --RArm
  -90,-45, -- Waist
  0, -90,
  0, -90,
  -60, -- Lidar pan
})*DEG_TO_RAD
assert(#servo.min_rad==nJoint,'Bad servo min_rad!')

servo.max_rad = vector.new({
  90,80, -- Head
  160,87,90,   0,     180,87,180, --LArm
  175,175,175,175,175,175, --LLeg
  175,175,175,175,175,175, --RLeg
  160,-0,90,   0,     180,87,180, --RArm
  90,45, -- Waist
  90,20,
  90,20,
  60, -- Lidar pan
})*DEG_TO_RAD
assert(#servo.max_rad==nJoint,'Bad servo max_rad!')
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
-- Export
Config.servo = servo
