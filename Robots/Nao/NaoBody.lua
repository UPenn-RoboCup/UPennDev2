--------------------------------
-- Body abstraction for THOR-OP
-- (c) 2013,2014 Stephen McGill, Seung-Joon Yi
--------------------------------

-- Utilities
local unix         = require'unix'
local vector       = require'vector'
local util         = require'util'
local Kinematics = require'NaoKinematics'

local Body = {}
local get_time = unix.time

--------------------------------
-- Shared memory layout
local indexHead = 1   -- Head: 1 2
local nJointHead = 2
local indexLArm = 3   --LArm: 3 4 5 6 7 8 9
local nJointLArm = 4
local indexLLeg = 7  --LLeg: 10 11 12 13 14 15
local nJointLLeg = 6
local indexRLeg = 13  --RLeg: 16 17 18 19 20 21
local nJointRLeg = 6
local indexRArm = 19  --RArm: 22 23 24 25 26 27 28
local nJointRArm = 4
local nJoint = 22
Body.nJoint = nJoint

local parts = {
	Head=vector.count(indexHead,nJointHead),
	LArm=vector.count(indexLArm,nJointLArm),
	LLeg=vector.count(indexLLeg,nJointLLeg),
	RLeg=vector.count(indexRLeg,nJointRLeg),
	RArm=vector.count(indexRArm,nJointRArm),
}
local inv_parts = {}
for name,list in pairs(parts) do
	for _,idx in ipairs(list) do inv_parts[idx]=name end
end

function Body.set_waist_torque_enable(val)
end
function Body.request_waist_position(val)
end
function Body.get_waist_command_position(val)
end
function Body.get_waist_position(val)
end

----------------------
-- Webots compatibility
if IS_WEBOTS then
  
  local webots = require'webots'
  local ImageProc = require'ImageProc'
  
  -- Default configuration (toggle during run time)
  local ENABLE_CAMERA = true
  local ENABLE_IMU    = true
  local ENABLE_POSE   = false
  

  local jointNames = {
    "HeadYaw", "HeadPitch",
    "LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll",
    "LHipYawPitch", "LHipRoll", "LHipPitch", "LKneePitch", "LAnklePitch", "LAnkleRoll",
    "RHipYawPitch", "RHipRoll", "RHipPitch", "RKneePitch", "RAnklePitch", "RAnkleRoll",
    "RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll",
  }
  assert(nJoint==#jointNames,'bad jointNames!')
  
  local actuator = {}
  actuator.command = {}
  actuator.velocity = {}
  actuator.position = {}
  actuator.hardness = {}
  for i = 1,nJoint do
    actuator.command[i] = 0
    actuator.velocity[i] = 0
    actuator.position[i] = 0
    actuator.hardness[i] = 0
  end
	
  -- Start the system
  webots.wb_robot_init()
  -- Acquire the timesteps
  local timeStep = webots.wb_robot_get_basic_time_step()
  local camera_timeStep = math.max(33,timeStep)
  local lidar_timeStep = math.max(25,timeStep)
  Body.timeStep = timeStep
  get_time = webots.wb_robot_get_time
  Body.get_time = webots.wb_robot_get_time
  
  -- For IMU use
  local tDelta = .001 * Body.timeStep
  local imuAngle = {0, 0};
  local aImuFilter = 1 - math.exp(-tDelta/0.5);

  -- Setup the webots tags
  local tags = {}

  -- Ability to turn on/off items
  local t_last_keypress = get_time()
  -- Enable the keyboard 100ms
  webots.wb_robot_keyboard_enable( 100 )
  local key_action = {
    c = function(override)
      if override~=nil then en=override else en=ENABLE_CAMERA==false end
      if en==false then
        print(util.color('CAMERA disabled!','yellow'))
        webots.wb_camera_disable(tags.camera_top)
        webots.wb_camera_disable(tags.camera_bottom)
        ENABLE_CAMERA = false
      else
        print(util.color('CAMERA enabled!','green'))
        webots.wb_camera_enable(tags.camera_top,camera_timeStep)
        webots.wb_camera_enable(tags.camera_bottom,camera_timeStep)
        ENABLE_CAMERA = true
      end
    end,
		p = function(override)
			if override~=nil then en=override else en=ENABLE_POSE==false end
      if en==false then
        print(util.color('POSE disabled!','yellow'))
        webots.wb_gps_disable(tags.gps)
        if tags.compass then
          webots.wb_compass_disable(tags.compass)
        end
        if webots.wb_inertial_unit_disable then
				  webots.wb_inertial_unit_disable(tags.inertialunit)
        end
        ENABLE_POSE = false
      else
        print(util.color('POSE enabled!','green'))
        webots.wb_gps_enable(tags.gps, timeStep)
        if tags.compass then
	  		  webots.wb_compass_enable(tags.compass, timeStep)
        end
        if webots.wb_inertial_unit_enable then
				  webots.wb_inertial_unit_enable(tags.inertialunit, timeStep)
        end
        ENABLE_POSE = true
      end
    end,
		i = function(override)
			if override~=nil then en=override else en=ENABLE_IMU==false end
      if en==false then
        print(util.color('IMU disabled!','yellow'))
        webots.wb_accelerometer_disable(tags.accelerometer)
  			webots.wb_gyro_disable(tags.gyro)
        ENABLE_IMU = false
      else
        print(util.color('IMU enabled!','green'))
        webots.wb_accelerometer_enable(tags.accelerometer, timeStep)
  			webots.wb_gyro_enable(tags.gyro, timeStep)
        ENABLE_IMU = true
      end
    end,
		f = function(override)
			if override~=nil then en=override else en=ENABLE_FSR==false end
      if en==false then
        print(util.color('FSR disabled!','yellow'))
        webots.wb_touch_sensor_disable(tags.l_fsr)
  			webots.wb_touch_sensor_disable(tags.r_fsr)
        ENABLE_FSR = false
      else
        print(util.color('FSR enabled!','green'))
        webots.wb_touch_sensor_enable(tags.l_fsr, timeStep)
  			webots.wb_touch_sensor_enable(tags.r_fsr, timeStep)
        ENABLE_FSR = true
      end
    end,
  }
	
	function Body.entry ()

		-- Grab the tags from the joint names
		tags.joints = {}
		for i,v in ipairs(jointNames) do
      local tag = webots.wb_robot_get_device(v)
			if tag>0 then
        -- For webots versions
        if webots.wb_motor_enable_position then
          webots.wb_motor_enable_position(tag, timeStep)
          webots.wb_motor_set_velocity(tag, 0.5);
        else
          webots.wb_servo_enable_position(tag, timeStep);
        end
        tags.joints[i] = tag
			end
		end

		-- Add Sensor Tags
		tags.accelerometer = webots.wb_robot_get_device("accelerometer")
		tags.gyro = webots.wb_robot_get_device("gyro")
		tags.gps = webots.wb_robot_get_device("gps")
		tags.inertialunit = webots.wb_robot_get_device("inertial unit")
    tags.us1 = webots.wb_robot_get_device("USSensor1")
    tags.us2 = webots.wb_robot_get_device("USSensor2")
    tags.us3 = webots.wb_robot_get_device("USSensor3")
    tags.us4 = webots.wb_robot_get_device("USSensor4")
		tags.camera_top = webots.wb_robot_get_device("CameraTop")
    tags.camera_bottom = webots.wb_robot_get_device("CameraBottom")
		tags.l_fsr = webots.wb_robot_get_device("LFsr")
    tags.r_fsr = webots.wb_robot_get_device("RFsr")
		
		-- Enable or disable the sensors
		key_action.i(ENABLE_IMU)
		key_action.p(ENABLE_POSE)
		key_action.c(ENABLE_CAMERA)
		key_action.f(ENABLE_FSR)

		-- Take a step to get some values
		webots.wb_robot_step(timeStep)
    webots.wb_robot_step(timeStep)

	end
  
  local camera_top, camera_bottom
  function Body.get_img_top ()
    local w = webots.wb_camera_get_width(tags.camera_top)
    local h = webots.wb_camera_get_height(tags.camera_top)
    return ImageProc.rgb_to_yuyv(webots.to_rgb(tags.camera_top), w, h)
  end
  function Body.get_img_bottom ()
    local w = webots.wb_camera_get_width(tags.camera_bottom)
    local h = webots.wb_camera_get_height(tags.camera_bottom)
    return ImageProc.rgb_to_yuyv(webots.to_rgb(tags.camera_bottom), w, h)
  end
  
	function Body.update ()

    local t = Body.get_time()

		-- Set actuator commands from shared memory
		for idx, jtag in ipairs(tags.joints) do
      
			local cmd = actuator.command[idx]
			local pos = actuator.position[idx]
      local vel = 0
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
			if jtag>0 and actuator.hardness[idx] > 0 then
        local pos = new_pos
        --SJ: Webots is STUPID so we should set direction correctly to prevent flip
        local val
        if webots.wb_motor_get_position then
          val = webots.wb_motor_get_position(jtag)
        else
          val = webots.wb_servo_get_position(jtag)
        end
        if pos>val+math.pi then
          pos = pos - 2 * math.pi
        elseif pos<val-math.pi then
          pos = pos + 2 * math.pi
        end
        if webots.wb_motor_set_position then
          webots.wb_motor_set_position(jtag, pos)
        else
          webots.wb_servo_set_position(jtag, pos)
        end
      end
		end --for

		-- Step the simulation, and shutdown if the update fails
		if webots.wb_robot_step(Body.timeStep) < 0 then os.exit() end

    if ENABLE_IMU then
      local accel = webots.wb_accelerometer_get_values(tags.accelerometer)
      local gyro = webots.wb_gyro_get_values(tags.gyro)
      -- Accelerometer data (verified)
      --[[
      jcm.sensorPtr.accelerometer[1] = (accel[1]-512)/128
      jcm.sensorPtr.accelerometer[2] = (accel[2]-512)/128
      jcm.sensorPtr.accelerometer[3] = (accel[3]-512)/128
      --]]
      -- Gyro data (verified)
      --[[
      jcm.sensorPtr.gyro[1] = -(gyro[1]-512)/512*39.24
      jcm.sensorPtr.gyro[2] = -(gyro[2]-512)/512*39.24
      jcm.sensorPtr.gyro[3] = (gyro[3]-512)/512*39.24
      --]]
      
      local gAccel = 9.80;
      local accX = accel[2]/gAccel;
      local accY = -accel[1]/gAccel;
      if ((accX > -1) and (accX < 1) and (accY > -1) and (accY < 1)) then
        imuAngle[1] = imuAngle[1] + aImuFilter*(math.asin(accX) - imuAngle[1]);
        imuAngle[2] = imuAngle[2] + aImuFilter*(math.asin(accY) - imuAngle[2]);
      end
      
    end

    -- FSR
    if ENABLE_FSR then
      jcm.sensorPtr.lfoot[1] = webots.wb_touch_sensor_get_value(tags.l_fsr)*4
      jcm.sensorPtr.rfoot[1] = webots.wb_touch_sensor_get_value(tags.r_fsr)*4
    end

    -- GPS and compass data
    -- Webots x is our y, Webots y is our z, Webots z is our x,
    -- Our x is Webots z, Our y is Webots x, Our z is Webots y
    if ENABLE_POSE then
      local gps = webots.wb_gps_get_values(tags.gps)
      local rpy = webots.wb_inertial_unit_get_roll_pitch_yaw(tags.inertialunit)
      --local angle   = math.atan2( compass[3], compass[1] )
      local angle = 0
      local pose    = vector.pose{gps[3], gps[1], angle}
      --wcm.set_robot_pose( pose )
      wcm.set_robot_pose_gps( pose )

      --[[
      print('rpy',unpack(rpy) )
      print('gps',unpack(gps) )
      print('compass',unpack(compass) )
      print('pose', pose )
      print()
      --]]
    end

    -- Grab keyboard input, for modifying items
    local key_code = webots.wb_robot_keyboard_get_key()
    local key_char = string.char(key_code)
    local key_char_lower = string.lower(key_char)
		local key_toggle = key_action[key_char_lower]
    if key_toggle and t-t_last_keypress>1 then
      key_toggle()
      t_last_keypress = t
    end

	end -- update

	Body.exit = function()
	end
	Body.tags = tags
  
  local function get_sensor_position(index)
    if (index) then
      if webots.wb_motor_get_position then
        return webots.wb_motor_get_position(tags.joints[index]);
      else
        return webots.wb_servo_get_position(tags.joints[index]);
      end
    else
      local t = {};
      for i = 1,nJoint do
        if webots.wb_motor_get_position then
          t[i] = webots.wb_motor_get_position(tags.joints[i])
        else
          t[i] = webots.wb_servo_get_position(tags.joints[i])
        end
      end
      return t;
    end
  end
  Body.get_sensor_position = get_sensor_position
  
  local function set_actuator_command(a, index)
    index = index or 1;
    if (type(a) == "number") then
      actuator.command[index] = a
    else
      for i = 1,#a do
        actuator.command[index+i-1] = a[i]
      end
    end
  end
  Body.set_actuator_command = set_actuator_command
  
  local function get_actuator_command(index)
    if index then
      return actuator.command[index]
    else
      local t = {}
      for i = 1,nJoint do
        t[i] = actuator.command[i]
      end
      return t
    end
  end
  
  local function set_actuator_hardness(a, index)
    index = index or 1;
    if (type(a) == "number") then
      actuator.hardness[index] = a;
    else
      for i = 1,#a do
        actuator.hardness[index+i-1] = a[i];
      end
    end
  end
  
  local function set_actuator_velocity(a, index)
    index = index or 1;
    if (type(a) == "number") then
      actuator.velocity[index] = a;
    else
      for i = 1,#a do
        actuator.velocity[index+i-1] = a[i];
      end
    end
  end
  Body.set_actuator_velocity = set_actuator_velocity
  
  function Body.get_head_position()
    local q = get_sensor_position()
    return {unpack(q, indexHead, indexHead+nJointHead-1)}
  end
  function Body.get_larm_position()
    local q = get_sensor_position()
    return {unpack(q, indexLArm, indexLArm+nJointLArm-1)}
  end
  function Body.get_rarm_position()
    local q = get_sensor_position()
    return {unpack(q, indexRArm, indexRArm+nJointRArm-1)}
  end
  function Body.get_lleg_position()
    local q = get_sensor_position()
    return {unpack(q, indexLLeg, indexLLeg+nJointLLeg-1)}
  end
  function Body.get_rleg_position()
    local q = get_sensor_position()
    return {unpack(q, indexRLeg, indexRLeg+nJointRLeg-1)}
  end
  
  function Body.set_head_command(val)
    set_actuator_command(val, indexHead);
  end
  
  -- Totally unsure why this is here...
  local commanded_joint_angles = vector.zeros(nJoint)
  Body.commanded_joint_angles = commanded_joint_angles
  function Body.set_lleg_command(val)
    set_actuator_command(val, indexLLeg);
    for i=1,#val do
      commanded_joint_angles[6+i] = val[i];
    end
  end
  function Body.set_rleg_command(val)
    set_actuator_command(val, indexRLeg);
  end
  function Body.set_larm_command(val)
    set_actuator_command(val, indexLArm);
  end
  function Body.set_rarm_command(val)
    set_actuator_command(val, indexRArm);
  end
  function Body.set_waist_command(val)
  end
  
  function Body.get_head_command()
    local q = get_actuator_command()
    return {unpack(q, indexHead, indexHead+nJointHead-1)}
  end
  function Body.get_larm_command()
    local q = get_actuator_command()
    return {unpack(q, indexLArm, indexLArm+nJointLArm-1)}
  end
  function Body.get_rarm_command()
    local q = get_actuator_command()
    return {unpack(q, indexRArm, indexRArm+nJointRArm-1)}
  end
  function Body.get_lleg_command()
    local q = get_actuator_command()
    return {unpack(q, indexLLeg, indexLLeg+nJointLLeg-1)}
  end
  function Body.get_rleg_command()
    local q = get_actuator_command()
    return {unpack(q, indexRLeg, indexRLeg+nJointRLeg-1)}
  end
  function Body.get_waist_command()
    return {0,0}
  end
  
  function Body.set_head_torque_enable(val)
  end
  function Body.set_lleg_torque_enable(val)
  end
  function Body.set_rleg_torque_enable(val)
  end
  function Body.set_larm_torque_enable(val)
  end
  function Body.set_rarm_torque_enable(val)
  end
  function Body.set_waist_torque_enable(val)
  end
  
  function Body.request_head_position()
  end
  function Body.request_larm_position()
  end
  function Body.request_rarm_position()
  end
  function Body.request_lleg_position()
  end
  function Body.request_rleg_position()
  end
  
  function Body.set_body_hardness(val)
    if (type(val) == "number") then
      val = val*vector.ones(nJoint);
    end
    set_actuator_hardness(val);
  end
  function Body.set_head_hardness(val)
    if (type(val) == "number") then
      val = val*vector.ones(nJointHead);
    end
    set_actuator_hardness(val, indexHead);
  end
  function Body.set_larm_hardness(val)
    if (type(val) == "number") then
      val = val*vector.ones(nJointLArm);
    end
    set_actuator_hardness(val, indexLArm);
  end
  function Body.set_rarm_hardness(val)
    if (type(val) == "number") then
      val = val*vector.ones(nJointRArm);
    end
    set_actuator_hardness(val, indexRArm);
  end
  function Body.set_lleg_hardness(val)
    if (type(val) == "number") then
      val = val*vector.ones(nJointLLeg);
    end
    set_actuator_hardness(val, indexLLeg);
  end
  function Body.set_rleg_hardness(val)
    if (type(val) == "number") then
      val = val*vector.ones(nJointRLeg);
    end
    set_actuator_hardness(val, indexRLeg);
  end
  function Body.set_waist_hardness(val)
  end

  function Body.set_actuator_us()
  end
  function Body.get_sensor_usLeft()
    return vector.zeros(10);
  end
  function Body.get_sensor_usRight()
    return vector.zeros(10);
  end
  
  function Body.set_syncread_enable(val)
  end
  
  function Body.get_sensor_imuAngle(index)
    if not index then
      return imuAngle;
    else
      return imuAngle[index];
    end
  end
  
  function Body.get_sensor_imuGyrRPY()
    -- TODO: Reimplement this
    --[[
    imuGyrRaw = get_sensor_imuGyr();
    gyro_roll = -(imuGyrRaw[1]-gyro0[1]);
    gyro_pitch = -(imuGyrRaw[2]-gyro0[2]);
    gyrRPY = vector.new({gyro_roll, gyro_pitch, 0});
    return gyrRPY;
    --]]
    return vector.zeros(3)
  end
  
  function Body.get_battery_level()
    return 10;
  end
  
end

Body.Kinematics = Kinematics

return Body
