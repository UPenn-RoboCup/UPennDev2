-- THOR OP Body
print('webots?',_G['IS_WEBOTS'])
-- Dynamixel definitions
local libDynamixel = require'libDynamixel'

-- Shared memory for the joints
require'jcm'

-- Utilities
require'unix'
require'vector'
local function mod_angle(a)
	-- Reduce angle to [-pi, pi)
	local b = a % (2*math.pi)
	if b >= math.pi then return (b - 2*math.pi) end
	return b
end

local DEG_TO_RAD = math.pi/180
local RAD_TO_DEG = 180/math.pi

local Body = {}

--------------------------------
-- Shared memory layout

-- Real THOR-OP (Cenatur uses ankles for wheels, maybe?)
local indexHead = 1   -- Head: 1 2
local nJointHead = 2
local indexLArm = 3   --LArm: 3 4 5 6 7 8
local nJointLArm = 6 		
local indexLLeg = 9  --LLeg: 9 10 11 12 13 14
local nJointLLeg = 6
local indexRLeg = 15  --RLeg: 15 16 17 18 19 20
local nJointRLeg = 6
local indexRArm = 21  --RArm: 21 22 23 24 25 26
local nJointRArm = 6
local indexWaist = 27  --Waist: 27 28
local nJointWaist = 2
-- Auxiliary servos
-- 6 Fingers for gripping
-- 1 servo for lidar panning
local indexAux = 29
local nJointAux = 7
local indexLGrip = 29
local nLGrip = 3
local indexRGrip = 32
local nRGrip = 3
local nJoint = 35

assert(nJoint==(indexAux+nJointAux)-indexHead,'Bad joint counting!')

local jointNames = {
	"Neck","Head", -- Head
	-- Left Arm
	"ShoulderL", "ArmUpperL", "LeftShoulderYaw",
	"ArmLowerL","LeftWristYaw","LeftWristRoll",
	-- Left leg
	"PelvYL","PelvL","LegUpperL","LegLowerL","AnkleL","FootL",
	-- Right leg
	"PelvYR","PelvR","LegUpperR","LegLowerR","AnkleR","FootR",
	--Right arm
	"ShoulderR", "ArmUpperR", "RightShoulderYaw","ArmLowerR",
	"RightWristYaw","RightWristRoll",
	-- Waist
	"TorsoPitch","TorsoYaw",
	-- Gripper
	"l_wrist_grip1","l_wrist_grip2","l_wrist_grip3",
	"r_wrist_grip1","r_wrist_grip2","r_wrist_grip3",
	-- lidar movement
	"ChestLidar",
}
assert(nJoint==#jointNames,'bad jointNames!')
local parts = {
	['Head']=vector.count(indexHead,nJointHead),
	['LArm']=vector.count(indexLArm,nJointLArm),
	['LLeg']=vector.count(indexLLeg,nJointLLeg),
	['RLeg']=vector.count(indexRLeg,nJointRLeg),
	['RArm']=vector.count(indexRArm,nJointRArm),
	['Waist']=vector.count(indexWaist,nJointWaist),
	['Aux']=vector.count(indexAux,nJointAux)
}
local inv_parts = {}
for name,list in pairs(parts) do
	for i,idx in ipairs(list) do
		inv_parts[idx] = name
	end
end

--------------------------------
-- Servo parameters
local servo = {}

-- shm joint id to dynamixel motor id
servo.joint_to_motor={
	27,28,  --Head
	1,3,5,7,9,11, --LArm
	-- TODO: No legs!  Using fake id numbers for now
	20,51,52,53,54,55, -- left wheel
	19,61,62,63,64,65, -- right wheel
	2,4,6,8,10,12,  --RArm
	25,26,  -- Waist
	14,16,18, -- left gripper
	13,15,17, -- right gripper
	36, -- Lidar pan
}
assert(#servo.joint_to_motor==nJoint,'Bad servo id map!')

local motor_parts = {}
for name,list in pairs(parts) do
	motor_parts[name] = vector.zeros(#list)
	for i,idx in ipairs(list) do
		motor_parts[name][i] = servo.joint_to_motor[idx]
	end
end

-- Make the reverse map
servo.motor_to_joint={}
for j,m in ipairs(servo.joint_to_motor) do
	servo.motor_to_joint[m] = j
end

-- NOTE: Servo direction is webots/real robot specific
servo.direction = vector.new({
	1,1, -- Head
  1,1,1,-1,1,-1, --LArm
	-- TODO: No legs yet! Using fake directions for now
	1, 1,1,1,1,1, --LLeg
	1, 1,1,1,1,1, --LArm
	-1,1,1,1, 1,-1, --RArm
	1,1, -- Waist
	-- TODO: Check the gripper
	1,-1,1, -- left gripper
	1,-1,1, -- right gripper
	1, -- Lidar pan
})
assert(#servo.direction==nJoint,'Bad servo direction!')

--http://support.robotis.com/en/product/dynamixel_pro/control_table.htm#Actuator_Address_611
-- TODO: Use some loop based upon MX/NX
-- TODO: some pros are different
servo.steps = 2*vector.new({
	151875,151875, -- Head
	251000,251000,251000,251000,151875,151875, --LArm
	-- TODO: No legs yet! Using fake directions for now
	251000,251000,251000,251000,251000,251000, --LLeg
	251000,251000,251000,251000,251000,251000, --LArm
	251000,251000,251000,251000,151875,151875, --RArm
	251000,251000, -- Waist
	-- TODO: Check the gripper
	2048,2048,2048, -- left gripper
	2048,2048,2048, -- right gripper
	2048, -- Lidar pan
})

-- Convienence tables to go between steps and radians
servo.moveRange = 360 * DEG_TO_RAD * vector.ones(nJoint)
-- Step<-->Radian ratios
servo.to_radians = vector.zeros(nJoint)
servo.to_steps = vector.zeros(nJoint)
for i, nsteps in ipairs(servo.steps) do
  servo.to_steps[i] = nsteps / servo.moveRange[i]
  servo.to_radians[i] = servo.moveRange[i] / nsteps
end

-- Set the radians
servo.min_rad = vector.ones(nJoint) * -175 * DEG_TO_RAD
servo.max_rad = vector.ones(nJoint) * 175 * DEG_TO_RAD

-- Set the steps
servo.min_step = vector.zeros(nJoint)
servo.max_step = vector.zeros(nJoint)
servo.step_zero = vector.new(servo.steps) / 2
for i, nsteps in ipairs(servo.steps) do
  if nsteps~=4096 then servo.step_zero[i] = 0 end
end

-- TODO: Implement step bias
-- TODO: Offset
servo.step_bias = vector.zeros(nJoint)
servo.rad_bias = vector.zeros(nJoint)

for i, nsteps in ipairs(servo.steps) do
  servo.rad_bias[i] = servo.to_radians[i] * servo.step_bias[i]
	servo.max_step[i] = servo.steps[i] + servo.step_bias[i]
	servo.min_rad[i] = servo.to_radians[i]*servo.min_step[i]
	servo.max_rad[i] = servo.to_radians[i]*servo.max_step[i]
end

-- Add some bias to this finger, since it was mounted wrong...
-- TODO: Remount the finger...
if not IS_WEBOTS then
  servo.step_bias[33] = -920
end
-- Some limit overrides
-- Elbow should not go beyond some boundaries and has some offsets
local elR = indexLArm+4-1
servo.step_bias[elR] = 45 * DEG_TO_RAD * servo.to_steps[elR]
servo.min_rad[elR]  = -100 * DEG_TO_RAD
servo.max_rad[elR]  = 0
-- Elbow should not go beyond some boundaries and has some offsets
local elL = indexLArm+4-1
servo.step_bias[elL] = -45 * DEG_TO_RAD * servo.to_steps[elL]
servo.min_rad[elL]  = 0
servo.max_rad[elL]  = 100 * DEG_TO_RAD

-- Right Wrist should not go beyond some boundaries
servo.min_rad[25]  = -100*DEG_TO_RAD
servo.max_rad[25]  = 100*DEG_TO_RAD
servo.min_rad[26]  = -80*DEG_TO_RAD
servo.max_rad[26]  = 80*DEG_TO_RAD

-- Gripper has different min/max limits
for _,idx in ipairs( parts['Aux'] ) do
	servo.min_rad[idx] = -10*DEG_TO_RAD
	servo.max_rad[idx] = 30*DEG_TO_RAD
end

-- Add the bias in to the min/max helper tables
for i, min_rad in ipairs(servo.min_rad) do
  servo.min_step[i] = math.floor( min_rad * servo.to_steps[i] ) + servo.step_zero[i] + servo.step_bias[i]
end
for i, max_rad in ipairs(servo.max_rad) do
  servo.max_step[i] = math.floor( max_rad * servo.to_steps[i] ) + servo.step_zero[i] + servo.step_bias[i]
end
for i, bias in ipairs(servo.step_bias) do
  servo.rad_bias[i] = bias * servo.to_radians[i]
end

-- Radian to step, using offsets and biases
local make_joint_step = function( idx, radian, safe )
	local step = math.floor(servo.direction[idx] * radian * servo.to_steps[idx]
	+ servo.step_zero[idx] + servo.step_bias[idx])
	if not safe then return step end
  local safe_step = math.min(math.max(step, servo.min_step[idx]), servo.max_step[idx])
	return safe_step
end

-- Step to radian
local make_joint_radian = function( idx, step )
	local radian = servo.direction[idx] * servo.to_radians[idx] * 
	(step - servo.step_zero[idx] - servo.step_bias[idx])
	return radian
end

--------------------------------
-- Legacy API
Body.set_syncread_enable = function()
end

--------------------------------
-- Standard convenient antropomorphic functions to access jcm
for part,jlist in pairs( parts ) do
	local a = jlist[1]
	local b = jlist[#jlist]
	Body['get_'..part:lower()..'_position'] = function(idx)
		if idx then return jcm.sensorPtr.position[ jlist[idx] ] end
		return jcm.sensorPtr.position:table( a, b )
	end -- Get
	Body['set_'..part:lower()..'_command'] = function(val,idx)
		if type(val)=='number' then
			if idx then jcm.actuatorPtr.command[ jlist[idx] ] = val return end
			for _,l in ipairs(jlist) do
				jcm.actuatorPtr.command[l] = val
			end
		else
			for i,l in ipairs(jlist) do
				jcm.actuatorPtr.command[l] = val[i]
			end
		end -- if number
	end -- Set

  -- Legacy API
	-- TODO: Hardness should set some PID values
	Body['get_'..part:lower()..'_hardness'] = function(idx)
		return 0
	end
	Body['set_'..part:lower()..'_hardness'] = function(val,idx)
	end

end

--------------------------------
-- Additional Convenience functions to access jcm
for part,jlist in pairs(parts) do

	local a = jlist[1]
	local b = jlist[#jlist]

	Body['get_'..part:lower()..'_command'] = function(idx)
		if idx then return jcm.actuatorPtr.command[ jlist[idx] ] end
		return jcm.actuatorPtr.command:table( a, b )
	end -- Get
	Body['set_'..part:lower()..'_position'] = function(val,idx)
		if type(val)=='number' then
			if idx then jcm.sensorPtr.position[ list[idx] ] = val return end
			for _,jid in ipairs(jlist) do
				jcm.sensorPtr.position[jid] = val
			end
		else
			for i,jid in ipairs(jlist) do
				jcm.sensorPtr.position[jid] = val[i]
			end
		end -- if number
	end -- Set


	Body['get_'..part:lower()..'_torque_enable'] = function(idx)
		if idx then return jcm.actuatorPtr.torque_enable[ jlist[idx] ] end
		return jcm.actuatorPtr.torque_enable:table( a, b )
	end -- Get
	Body['set_'..part:lower()..'_torque_enable'] = function(val,idx)
		if type(val)=='number' then
			if idx then jcm.actuatorPtr.torque_enable[ jlist[idx] ] = val return end
			for _,l in ipairs(jlist) do
				jcm.actuatorPtr.torque_enable[l] = val
			end
		else
			for i,l in ipairs(jlist) do
				jcm.actuatorPtr.torque_enable[l] = val[i]
			end
		end -- if number
	end -- Set

	Body['get_'..part:lower()..'_load'] = function(idx)
		if idx then return jcm.sensorPtr.load[ list[idx] ] end
		return jcm.sensorPtr.load:table( a, b )
	end -- Get
	Body['set_'..part:lower()..'_load'] = function(val,idx)
		if type(val)=='number' then
			if idx then jcm.sensorPtr.load[ jlist[idx] ] = val return end
			for _,l in ipairs(jlist) do jcm.sensorPtr.load[l] = val end
		else
			for i,l in ipairs(jlist) do jcm.sensorPtr.load[l] = val[i] end
		end -- if number
	end -- Set

end

--------------------------------
-- Packet generators
-- NX packet generators
for k,v in pairs(libDynamixel.nx_registers) do
  local get_func = libDynamixel['get_nx_'..k]
  local set_func = libDynamixel['set_nx_'..k]
  
  for part,jlist in pairs(parts) do

    local a = jlist[1]
    local b = jlist[#jlist]
  
    local mlist = motor_parts[part]
    local jlist = parts[part]
    local a = jlist[1]
    local b = jlist[#jlist]
    Body['get_'..part:lower()..'_'..k..'_packet'] = function()
      return get_func(mlist)
    end
    Body['set_'..part:lower()..'_'..k..'_packet'] = function()
      local vals = jcm.actuatorPtr[k]:table( a, b )
      return set_func( mlist, vals )
    end
    if k=='command_position' then
      --print'overwriting command'
      Body['set_'..part:lower()..'_'..k..'_packet'] = function()
        -- Shm has radians
        local vals = jcm.actuatorPtr.command:table( a, b )
        -- Convert to 
        for i,idx in ipairs(jlist) do
          vals[i] = make_joint_step(idx,vals[i],true)
        end
        return set_func( mlist, vals )
      end
    end
  end--jlist
end
  
-- Gripper and lidar actuator are MX
-- TODO: make general somehow
for k,v in pairs(libDynamixel.mx_registers) do
	local get_func = libDynamixel['get_mx_'..k]
	local set_func = libDynamixel['set_mx_'..k]
	local mlist = motor_parts['Aux']
	local jlist = parts['Aux']
	local a = jlist[1]
	local b = jlist[#jlist]
	Body['get_aux_'..k..'_packet'] = function()
		return get_func(mlist)
	end
	Body['set_aux_'..k..'_packet'] = function()
		local vals = jcm.actuatorPtr[k]:table( a, b )
		return set_func( mlist, vals )
	end
	if k=='command' then
		--print'overwriting command'
		Body['set_aux_command_packet'] = function()
			-- Shm has radians
			local vals = jcm.actuatorPtr.command:table( a, b )
			-- Convert to 
			for i,idx in ipairs(jlist) do
				vals[i] = make_joint_step(idx,vals[i],true)
			end
			return set_func( mlist, vals )
		end
	end
end

-- TODO: should be in body or a Grip module?
-- Grip module may have more advanced techniques...
-- More gripper functions
Body.set_lgrip_percent = function( percent )
	percent = math.min(math.max(percent,0),1)
	-- Convex combo
	for idx=indexLGrip,indexLGrip+nLGrip-1 do
		local radian = percent*servo.min_rad[idx] + (1-percent)*servo.max_rad[idx]
		jcm.actuatorPtr.command[idx] = radian
	end
end
Body.set_rgrip_percent = function( percent )
	percent = math.min(math.max(percent,0),1)
	-- Convex combo
	for idx=indexRGrip,indexRGrip+nRGrip-1 do
		local radian = percent * servo.min_rad[idx] + (1-percent)*servo.max_rad[idx]
		jcm.actuatorPtr.command[idx] = radian
	end
end


----------------------
-- Body sensor readings
-- NOTE: Should just iterate through jcm...
-- jcm should be the API compliance test
for sensor, pointer in pairs(jcm.sensorPtr) do
	Body['set_sensor_'..sensor] = function(val,idx)
		if idx then pointer[idx] = val; return; end
		for i,v in ipairs(val) do pointer[i] = v end
	end
	Body['get_sensor_'..sensor] = function(idx)
		local s = pointer:table()
		if idx then return s[idx] end
		return s
	end
end

----------------------
-- Body actuator commands
-- NOTE: Should just iterate through jcm...
-- jcm should be the API compliance test
for actuator, pointer in pairs(jcm.actuatorPtr) do
	Body['set_actuator_'..actuator] = function(val,idx)
		if idx then pointer[idx] = val; return; end
		for i,v in ipairs(val) do pointer[i] = v end
	end
	Body['get_actuator_'..actuator] = function(idx)
		local s = pointer:table()
		if idx then return s[idx] end
		return s
	end
end

----------------------
-- Inverse Kinematics
local Kinematics = require'THOROPKinematics'

-- Check the error from a desired transform tr
-- to a forwards kinematics of in IK solution q
-- Tolerate a 1mm error in distance
local IK_TOLERANCE = 0.001
local function check_ik_error( q, tr )
	-- Transform from q (joint angles)
	local tr_q = Kinematics.r_arm_torso( q, tr )

	local position_error = math.sqrt(
	( tr_q[1]-tr[1] )^2 +
	( tr_q[2]-tr[2] )^2 +
	( tr_q[3]-tr[3] )^2 )

	local angle_error = math.sqrt(
	mod_angle( trRArm_target[4]-tr[4] )^2 +
	mod_angle( trRArm_target[5]-tr[5] )^2 +
	mod_angle( trRArm_target[6]-tr[6] )^2 )

	-- If within tolerance, return true
	if dist_pos<IK_TOLERANCE then return true end

	-- By default, return false, that we are out of tolerance
	return false

end

-- Take in a transform and output joint angles
Body.get_inverse_larm = function( tr )
	local qLArm = Body.get_larm_position()
	local qLArm_target = Kinematics.inverse_l_arm(tr, qLArm)
	--if not check_ik_error( qLArm_target, tr ) then return nil end
  return qLArm_target
end
Body.get_inverse_rarm = function( tr )
	local qRArm = Body.get_rarm_position()
	local qRArm_target = Kinematics.inverse_r_arm(tr, qRArm)
	--if not check_ik_error( qRArm_target, tr ) then return nil end
  return qRArm_target
end

-- Take in joint angles and output an {x,y,z,r,p,yaw} table
Body.get_forward_larm = function()
	local qLArm = Body.get_larm_position()
	local pLArm = Kinematics.l_arm_torso( qLArm )
  return pLArm
end
Body.get_forward_rarm = function()
	local qRArm = Body.get_rarm_position()
	local pRArm = Kinematics.r_arm_torso( qRArm )
  return pRArm
end

-- TODO: Write functions to modify transform in directions

----------------------
-- More standard api functions
Body.get_time = unix.time
Body.entry = function()
  -- Zero all joint requests
  for i=1,nJoint do Body.set_actuator_command(0,i) end
  for i=1,nJoint do Body.set_sensor_position(0,i) end
  for i=1,nJoint do Body.set_sensor_load(0,i) end
  for i=1,nJoint do Body.set_actuator_torque_enable(0,i) end
end
Body.update = function()
end
Body.exit = function()
end

----------------------
-- Webots compatibility
if IS_WEBOTS then
	local webots = require'webots'
	Body.get_time = webots.wb_robot_get_time

	servo.direction = vector.new({
		1,-1, -- Head
		1,-1,-1,1,-1,1, --LArm
		-- TODO: No legs yet!
		-1,-1,-1,-1,1,1, --LLeg
		-1,-1,1,1,-1,1, --LArm
		-1,-1,-1,-1,-1,1, --RArm
		-- TODO: Check the gripper
		1,1, -- Waist
		1,1,1, -- left gripper
		1,1,1, -- right gripper
		1, -- Lidar pan
	})
	servo.rad_bias = vector.new({
		0,0, -- head
		-90,-10,0,45,0,0,
		0,0,0,0,0,0,
		0,0,0,0,0,0,
		-90,10,0,45,0,0,
		0,0,
		0,0,0,
		0,0,0,
		0,
	})*math.pi/180


	-- Setup the webots tags
	local tags = {}

	Body.entry = function()
    
    -- Zero all joint requests
    for i=1,nJoint do Body.set_actuator_command(0,i) end

		-- Initialize the webots system
		webots.wb_robot_init()

		-- Grab the update time
		local timeStep = webots.wb_robot_get_basic_time_step()

		-- Grab the tags from the joint names
		tags.joints = {}
		for i,v in ipairs(jointNames) do
			tags.joints[i] = webots.wb_robot_get_device(v)
			if tags.joints[i]>0 then
				webots.wb_servo_enable_position(tags.joints[i], timeStep)
			else
				print(v,'not found')
			end
		end
		-- Add Sensor Tags
		-- Accelerometer
		tags.accelerometer = webots.wb_robot_get_device("Accelerometer")
		webots.wb_accelerometer_enable(tags.accelerometer, timeStep)
		-- Gyro
		tags.gyro = webots.wb_robot_get_device("Gyro")
		webots.wb_gyro_enable(tags.gyro, timeStep)
		-- TODO: Gps, compass, kinect
		--[[
		tags.gps = webots.wb_robot_get_device("zero")
		webots.wb_gps_enable(tags.gps, timeStep)
		-- Compass
		tags.compass = webots.wb_robot_get_device("compass")
		webots.wb_compass_enable(tags.compass, timeStep)
		-- Kinect
		tags.kinect = webots.wb_robot_get_device("kinect")
		webots.wb_camera_enable(tags.kinect, timeStep)
		--]]

		-- Take a step to get some values
		webots.wb_robot_step(timeStep)
		Body.timeStep = timeStep
		-- Filter the IMU a bit?
		-- TODO: Should use Yida's IMU filter
		--imuAngle = {0, 0, 0}
		--aImuFilter = 1 - math.exp(-tDelta/0.5)

	end
	Body.update = function()

		local tDelta = .001 * Body.timeStep
		-- Set actuator commands from shared memory
		for i, jtag in ipairs(tags.joints) do
			local cmd = Body.get_actuator_command(i)
			local pos = Body.get_sensor_position(i)
			-- TODO: What is velocity?
			local vel = 0 or Body.get_actuator_velocity(i)
			local en = 1 or Body.get_actuator_torque_enable(i)
			local deltaMax = tDelta * vel
			-- Only update the joint if the motor is torqued on

			-- If the joint is moving
			-- Clamp the difference between commanded and actuated
			-- so that we don't have huge jumped
			-- NOTE: This *should* be handled by the simulator?
			local new_pos = cmd
			if vel > 0 then
				local delta = cmd - pos
				if delta > deltaMax then
					delta = deltaMax
				elseif delta < -deltaMax then
					delta = -deltaMax
				end
				new_pos = pos + delta
			end

			-- Only set in webots if Torque Enabled
			if en > 0 and jtag>0 then
        webots.wb_servo_set_position(jtag, servo.direction[i] * new_pos - servo.rad_bias[i])
      end
		end



		-- Step the simulation, and shutdown if the update fails
		if webots.wb_robot_step(Body.timeStep) < 0 then os.exit() end

		-- Update the sensor readings
		-- TODO: Verify the compass
		--[[
		local compass = webots.wb_compass_get_values(tags.compass)
		-- Swap webots coordinates into our coordinates
		Body.set_sensor_compass( {compass[1],-compass[2],compass[3]} )
		--]]

		-- Accelerometer data (verified)
		local accel = webots.wb_accelerometer_get_values(tags.accelerometer)
		Body.set_sensor_accelerometer( 
		{(accel[1]-512)/128,(accel[2]-512)/128,-(accel[3]-512)/128}
		)
		-- Gyro data (verified)
		local gyro = webots.wb_gyro_get_values(tags.gyro)
		Body.set_sensor_gyro( 
		{(gyro[1]-512)/0.273, (gyro[2]-512)/0.273,(gyro[3]-512)/0.273}
		)

		-- Update the sensor readings of the joint positions
		-- TODO: If a joint is not found?
		for idx, jtag in ipairs(tags.joints) do
			if jtag>0 then
				local val = webots.wb_servo_get_position( jtag )
				local rad = servo.direction[idx] * val - servo.rad_bias[idx]
				Body.set_sensor_position( rad, idx )
			end
		end

	end

	Body.exit = function()
	end

	Body.tags = tags

end -- webots check

-- Exports for use in other functions

-- Real THOR-OP (Cenatur uses ankles for wheels, maybe?)
Body.indexHead = 1   -- Head: 1 2
Body.nJointHead = 2
Body.indexLArm = 3   --LArm: 5 6 7 8 9 10
Body.nJointLArm = 6 		
Body.indexLLeg = 9  --LLeg: 11 12 13 14 15 16
Body.nJointLLeg = 6
Body.indexRLeg = 15  --RLeg: 17 18 19 20 21 22
Body.nJointRLeg = 6
Body.indexRArm = 21  --RArm: 23 24 25 26 27 28
Body.nJointRArm = 6
Body.indexWaist = 27  --Waist: 3 4
Body.nJointWaist = 2
-- Auxiliary servos
-- 6 Fingers for gripping
-- 1 servo for lidar panning
Body.indexAux = 29
Body.nJointAux = 7
Body.indexLGrip = 29
Body.nLGrip = 3
Body.indexRGrip = 32
Body.nRGrip = 3

Body.nJoint = nJoint
Body.jointNames = jointNames
Body.parts = parts
Body.inv_parts = inv_parts
Body.motor_parts = motor_parts
Body.servo = servo
Body.make_joint_step = make_joint_step
Body.make_joint_radian = make_joint_radian

return Body
