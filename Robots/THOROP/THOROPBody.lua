--------------------------------
-- Body abstraction for THOR-OP
-- (c) 2013,2014 Stephen McGill, Seung-Joon Yi
--------------------------------
--assert(ffi, 'Need LuaJIT to run. Lua support in the future')

-- Utilities
local vector = require'vector'
local util = require'util'
local si = require'simple_ipc'
local Kinematics = require'THOROPKinematics'
local mpack  = require'msgpack.MessagePack'.pack
require'dcm'

local Body = {}
local dcm_ch = si.new_publisher'dcm!'
local get_time = require'unix'.time
local vslice = vector.slice

-- Body sensors
local nx_registers = require'libDynamixel'.nx_registers
for sensor, n_el in pairs(dcm.sensorKeys) do
	--local cur = dcm['get_sensor_'..sensor]()
	--local n_el = type(cur)=='table' and #cur or 1
  local is_motor = nx_registers[sensor]
  local ptr, ptr_t
  if dcm.sensorPtr then
    ptr = dcm.sensorPtr[sensor]
    ptr_t = dcm.tsensorPtr[sensor]
  end
  local get = function(idx1, idx2, needs_wait)
		local start, stop = idx1 or 1, idx2 or n_el
    if is_motor and needs_wait then
			local ids = {}
			for id = start, stop do ids[id] = true end
			dcm_ch:send(mpack({rd_reg=sensor, ids=ids}))
    end
  	-- For cdata, use -1
		-- Return the time of the reading
	  return vslice(ptr, start-1, stop-1), vslice(ptr_t, start-1, stop-1)
	end
  
  Body['get_'..sensor] = get
  -- Anthropomorphic access to dcm
	-- TODO: get_lleg_rpy is illegal, for instance
  for part, jlist in pairs(Config.parts) do
	  local not_synced = sensor~='position'
    local idx1, idx2 = jlist[1], jlist[#jlist]
    Body['get_'..part:lower()..'_'..sensor] = function(idx, skip_wait)
			return get(idx1, idx2, not_synced and not skip_wait)
    end -- Get
  end
	-- End anthropomorphic
end

-- Body actuators
for actuator, n_el in pairs(dcm.actuatorKeys) do
	-- Only command_position is constantly synced
	-- Other commands need to be specially sent to the Body
  -- TODO: Check the torque usage in NX motors...
	local not_synced = not (actuator=='command_position' or actuator=='command_torque')
  local ptr = dcm.actuatorPtr and dcm.actuatorPtr[actuator]
  local idx
	local function set(val, idx1, idx2)
		local changed_ids = {}
		-- cdata is -1
		if idx2 then
			if type(val)=='number' then
				for idx=idx1, idx2 do
					changed_ids[idx] = true
					ptr[idx - 1] = val
				end
			else
				for i,v in ipairs(val) do
					idx = idx1 + i - 1
					changed_ids[idx] = true
					if idx>idx2 then break else ptr[idx - 1] = v end
				end
			end
		elseif idx1 then
			if type(val)=='number' then
				changed_ids[idx1] = true
				ptr[idx1 - 1] = val
			else
				for i, v in ipairs(val) do
					idx = idx1 + i - 1
					changed_ids[idx] = true
					ptr[idx - 1] = v
				end
			end
		else
			-- No index means set all actuators... Uncommon
			if type(val)=='number' then
				for i=0, n_el-1 do
					changed_ids[i + 1] = true
					ptr[i] = val
				end
			else
				for i, v in ipairs(val) do
					changed_ids[i] = true
					ptr[i - 1] = v
				end
			end
		end
		-- Send msg to the dcm, just string of the id
		if not_synced then
			dcm_ch:send(mpack({wr_reg=actuator, ids=changed_ids}))
		end
	end
	local function get(idx1, idx2)
		idx1 = idx1 or 1
		idx2 = idx2 or n_el
		-- For cdata, use -1
		return vslice(ptr, idx1 - 1, idx2 - 1)
	end
	-- Export
  Body['set_'..actuator] = set
  Body['get_'..actuator] = get
  --------------------------------
  -- Anthropomorphic access to dcm
  -- TODO: Do not use string concatenation to call the get/set methods of Body
  for part, jlist in pairs(Config.parts) do
		local idx1, idx2, idx = jlist[1], jlist[#jlist], nil
		Body['get_'..part:lower()..'_'..actuator] = function(idx)
			if idx then return get(jlist[idx]) else return get(idx1, idx2) end
		end
		Body['set_'..part:lower()..'_'..actuator] = function(val, i)
			if i then
        return set(val, jlist[i])
      else
        -- Check the ankle on a full set of lleg/rleg
        -- Do not set the last 2 (ankle) values
        --[[
        if part=='LLeg' then
          if (val[6]>0 and val[5]>0) or (val[6]>0 and val[5]>0)
          then
            return set(val, idx1, idx2-2)
          end
        elseif part=='RLeg' then
          if (val[6]>0 and val[5]>0) or (val[6]>0 and val[5]>0)
          then
            return set(val, idx1, idx2-2)
          end
        end
        --]]
        return set(val, idx1, idx2)
      end
		end
  end
	-- End anthropomorphic
end

function Body.entry()
end

function Body.update()
end

function Body.exit()
end

---
-- Special functions
---
function Body.enable_read(chain)
  dcm_ch:send(mpack({bus=chain,key='enable_read', val=true}))
end
function Body.disable_read(chain)
  dcm_ch:send(mpack({bus=chain,key='enable_read', val=false}))
end

----------------------
-- Webots compatibility
if IS_WEBOTS then
	require'wcm'
	local WebotsBody
  local torch = require'torch'
  local webots = require'webots'
	local ImageProc = require'ImageProc'
  
  Body.enable_read = function(chain)
  end
  Body.disable_read = function(chain)
  end

	local nJoint = Config.nJoint
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
  assert(nJoint==#jointNames,'bad jointNames!')

  local servo = Config.servo

  -- Default configuration (toggle during run time)
  local ENABLE_LOG, t_log = false, 0
  if ENABLE_LOG then
  	libLog = require'libLog'
  	logger = libLog.new('yuyv', true)
  end

  -- Added to Config rather than hard-code 
	local ENABLE_CAMERA, NEXT_CAMERA = Config.sensors.head_camera, 0
  local ENABLE_CHEST_LIDAR, NEXT_CHEST_LIDAR  = Config.sensors.chest_lidar, 0
  local ENABLE_HEAD_LIDAR, NEXT_HEAD_LIDAR = Config.sensors.head_lidar, 0
  local ENABLE_FSR = Config.sensors.fsr
  local ENABLE_FT = Config.sensors.ft
  local ENABLE_KINECT = Config.sensors.kinect
  local ENABLE_POSE = true
  local ENABLE_IMU = true

  -- Start the system
  webots.wb_robot_init()
  -- Acquire the timesteps
  local timeStep = webots.wb_robot_get_basic_time_step()
  local camera_timeStep = math.max(Config.camera_timestep or 33, timeStep)
  local lidar_timeStep = math.max(Config.lidar_timestep or 25, timeStep)

  Body.timeStep = timeStep
  get_time = webots.wb_robot_get_time

  -- Setup the webots tags
  local tags = {}  
  local t_last_error = -math.huge

  tags.receiver = webots.wb_robot_get_device("receiver")
	if tags.receiver>0 then
		webots.wb_receiver_enable(tags.receiver, timeStep)
		webots.wb_receiver_set_channel(tags.receiver, 13)
	end
    
  -- Ability to turn on/off items
  local t_last_keypress = get_time()
  -- Enable the keyboard 100ms
  webots.wb_robot_keyboard_enable(100)
  local key_action = {
		h = function(override)
			if override~=nil then en=override else en=ENABLE_HEAD_LIDAR==false end
      if en==false and tags.head_lidar then
        print(util.color('HEAD_LIDAR disabled!','yellow'))
        webots.wb_camera_disable(tags.head_lidar)
        ENABLE_HEAD_LIDAR = false
      elseif tags.head_lidar then
        print(util.color('HEAD_LIDAR enabled!','green'))
        webots.wb_camera_enable(tags.head_lidar,lidar_timeStep)
				NEXT_HEAD_LIDAR = get_time() + lidar_timeStep / 1000
        ENABLE_HEAD_LIDAR = true
      end
    end,
    l = function(override)
			if override~=nil then en=override else en=ENABLE_CHEST_LIDAR==false end
      if en==false and tags.chest_lidar then
        print(util.color('CHEST_LIDAR disabled!','yellow'))
        webots.wb_camera_disable(tags.chest_lidar)
        ENABLE_CHEST_LIDAR = false
      elseif tags.chest_lidar then
        print(util.color('CHEST_LIDAR enabled!','green'))
        webots.wb_camera_enable(tags.chest_lidar,lidar_timeStep)
				NEXT_CHEST_LIDAR = get_time() + lidar_timeStep / 1000
        ENABLE_CHEST_LIDAR = true
      end
    end,
    c = function(override)
      if override~=nil then en=override else en=ENABLE_CAMERA==false end
      if en==false then
        print(util.color('CAMERA disabled!','yellow'))
        webots.wb_camera_disable(tags.head_camera)
        ENABLE_CAMERA = false
      else
        print(util.color('CAMERA enabled!','green'))
        webots.wb_camera_enable(tags.head_camera,camera_timeStep)
				NEXT_CAMERA = get_time() + camera_timeStep / 1000
        ENABLE_CAMERA = true
      end
    end,
		p = function(override)
			if override~=nil then en=override else en=ENABLE_POSE==false end
      if en==false then
        print(util.color('POSE disabled!','yellow'))
        webots.wb_gps_disable(tags.gps)
	  		webots.wb_compass_disable(tags.compass)
				webots.wb_inertial_unit_disable(tags.inertialunit)
        ENABLE_POSE = false
      else
        print(util.color('POSE enabled!','green'))
        webots.wb_gps_enable(tags.gps, timeStep)
	  		webots.wb_compass_enable(tags.compass, timeStep)
				webots.wb_inertial_unit_enable(tags.inertialunit, timeStep)
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
		t = function(override)
			if override~=nil then en=override else en=ENABLE_FT==false end
      if en==false then
        print(util.color('FT disabled!','yellow'))
        webots.wb_touch_sensor_disable(tags.l_ft)
  			webots.wb_touch_sensor_disable(tags.r_ft)
        ENABLE_FT = false
      else
        print(util.color('FT enabled!','green'))
        webots.wb_touch_sensor_enable(tags.l_ft, timeStep)
  			webots.wb_touch_sensor_enable(tags.r_ft, timeStep)
        webots.wb_motor_enable_force_feedback(tags.jointsByName.FootR, timeStep)
        webots.wb_motor_enable_force_feedback(tags.jointsByName.FootL, timeStep)
        webots.wb_motor_enable_force_feedback(tags.jointsByName.AnkleR, timeStep)
        webots.wb_motor_enable_force_feedback(tags.jointsByName.AnkleL, timeStep)
        if tags.left_ankle_yaw > 0 then
          webots.wb_motor_enable_force_feedback(tags.left_ankle_yaw, timeStep)
        end
        if tags.right_ankle_yaw > 0 then
          webots.wb_motor_enable_force_feedback(tags.right_ankle_yaw, timeStep)
        end
        ENABLE_FT = true
      end
    end,
  }
	-- Check if we are using the OLD api
	local OLD_API = webots.wb_device_get_type(webots.wb_robot_get_device(jointNames[1]))==89

	local set_pos, get_pos = webots.wb_motor_set_position, webots.wb_motor_get_position
	if OLD_API then
		set_pos = webots.wb_servo_set_position
		get_pos = webots.wb_servo_get_position
	end
  local PID_P = 32 * vector.ones(nJoint)
	function Body.entry()
    -- Request @ t=0 to always be earlier than position reads

		-- Grab the tags from the joint names
		tags.joints, tags.jointsByName = {}, {}
		for i,v in ipairs(jointNames) do
      local tag = webots.wb_robot_get_device(v)
			tags.joints[i] = tag
      tags.jointsByName[v] = tag
			if tag > 0 then
				if OLD_API then
					webots.wb_servo_enable_position(tag, timeStep)
	        webots.wb_servo_set_velocity(tag, 4)
				else
					webots.wb_motor_enable_position(tag, timeStep)
				end
			end
		end
    -- Add the foot yaw, giving the Torque around the z axis
    tags.left_ankle_yaw = webots.wb_robot_get_device('left_ankle_yaw')
    tags.right_ankle_yaw = webots.wb_robot_get_device('right_ankle_yaw')

		-- Add Sensor Tags
		tags.accelerometer = webots.wb_robot_get_device("Accelerometer")
		tags.gyro = webots.wb_robot_get_device("Gyro")
		tags.gps = webots.wb_robot_get_device("GPS")
		tags.compass = webots.wb_robot_get_device("Compass")
		tags.inertialunit = webots.wb_robot_get_device("InertialUnit")
		tags.head_camera = webots.wb_robot_get_device("HeadCamera")

    if Config.sensors.chest_lidar then
      tags.chest_lidar = webots.wb_robot_get_device("ChestLidar")
    end		
    if Config.sensors.head_lidar then
      tags.head_lidar = webots.wb_robot_get_device("HeadLidar")
    end
    if Config.sensors.fsr then
      tags.l_fsr = webots.wb_robot_get_device("L_FSR")
      tags.r_fsr = webots.wb_robot_get_device("R_FSR")
    end
    if Config.sensors.ft then
			tags.l_ft = webots.wb_robot_get_device("LAnkle_force")
      tags.r_ft = webots.wb_robot_get_device("RAnkle_force")
    end
    
		-- Enable or disable the sensors
		key_action.i(ENABLE_IMU)
		key_action.p(ENABLE_POSE)
		if ENABLE_CAMERA then key_action.c(ENABLE_CAMERA) end
		if ENABLE_CHEST_LIDAR then key_action.l(ENABLE_CHEST_LIDAR) end
		if ENABLE_HEAD_LIDAR then key_action.h(ENABLE_HEAD_LIDAR) end
		if ENABLE_FT then key_action.t(ENABLE_FT) end

		-- Take a step to get some values
		webots.wb_robot_step(timeStep)
    
    -- PID setting
    Body.set_position_p(PID_P)

		local rad, val
		local positions = vector.zeros(nJoint)
    
    for idx, jtag in ipairs(tags.joints) do
      if jtag>0 then
        -- Update the PID if necessary
        if not OLD_API and webots.wb_motor_set_control_pid then
          webots.wb_motor_set_control_pid(jtag, PID_P[idx], 0, 0)
        end
				val = get_pos(jtag)
        rad = servo.direction[idx] * val - servo.rad_offset[idx]
				rad = rad==rad and rad or 0
				positions[idx] = rad
      end
    end
		dcm.set_sensor_position(positions)
		dcm.set_actuator_command_position(positions)
		WebotsBody = require'WebotsBody'
		WebotsBody.entry()
	end

	function Body.update()

    local t = get_time()
    --Body.update_finger(timeStep)

		-- Set actuator commands from shared memory
		local cmds = Body.get_command_position()
		local poss = Body.get_position()
		for idx, jtag in ipairs(tags.joints) do
			local cmd, pos = cmds[idx], poss[idx]

			-- TODO: What is velocity?
			local vel = 0 or Body.get_command_velocity()[idx]
			local en  = 1 or Body.get_torque_enable()[idx]
			-- Only update the joint if the motor is torqued on

			-- If the joint is moving
			-- Clamp the difference between commanded and actuated
			-- so that we don't have huge jumped
			-- NOTE: This *should* be handled by the simulator?
			--[[
			local deltaMax = timeStep * vel
			local new_pos = cmd
			if vel > 0 then
        local delta = util.mod_angle(cmd - pos)
				if delta > deltaMax then
					delta = deltaMax
				elseif delta < -deltaMax then
					delta = -deltaMax
				end
				new_pos = pos + delta
			end
			--]]

			if en>0 and jtag>0 then
        -- Update the PID
        if not OLD_API then
          local new_P, old_P = Body.get_position_p()[idx], PID_P[idx]
          if new_P ~= old_P then
            --print('UPDATE P!', idx, old_P, new_P)
            PID_P[idx] = new_P
            webots.wb_motor_set_control_pid(jtag, new_P, 0, 0)
          end
        end
        
        local rad = servo.direction[idx] * (cmd + servo.rad_offset[idx])
        set_pos(jtag, rad)
--SJ: Webots is STUPID so we should set direction correctly to prevent flip        
--[[        
        local val = get_pos(jtag)
        if pos > val + math.pi then
					rad = rad - 2 * math.pi
        elseif rad < val - math.pi then
					rad = rad + 2 * math.pi
        end
				rad = rad==rad and rad or 0
        set_pos(jtag, rad)
--]]
				--Fixed
				--[[
        local val = get_pos(jtag)
        local delta = util.mod_angle(rad-val)
        set_pos(jtag, rad+delta)
				--]]
      end
		end --for

		-- Step the simulation, and shutdown if the update fails
		if webots.wb_robot_step(Body.timeStep) < 0 then os.exit() end
		t = get_time()

    if ENABLE_IMU then
      -- Accelerometer data (verified)
      local accel = webots.wb_accelerometer_get_values(tags.accelerometer)
      dcm.sensorPtr.accelerometer[0] = (accel[1]-512)/128
      dcm.sensorPtr.accelerometer[1] = (accel[2]-512)/128
      dcm.sensorPtr.accelerometer[2] = (accel[3]-512)/128
      -- Gyro data (verified)
      local gyro = webots.wb_gyro_get_values(tags.gyro)
      dcm.sensorPtr.gyro[0] = -(gyro[1]-512)/512*39.24
      dcm.sensorPtr.gyro[1] = -(gyro[2]-512)/512*39.24
      dcm.sensorPtr.gyro[2] = (gyro[3]-512)/512*39.24
    end

    -- FSR
    if ENABLE_FSR then
      dcm.sensorPtr.lfoot[0] = webots.wb_touch_sensor_get_value(tags.l_fsr)
      dcm.sensorPtr.rfoot[0] = webots.wb_touch_sensor_get_value(tags.r_fsr)
    end
		
		-- F/T sensor
    if ENABLE_FT then
			local l_ft = Body.get_lfoot()
			l_ft[2], l_ft[3], l_ft[1] = unpack(webots.wb_touch_sensor_get_values(tags.l_ft))
      l_ft[4] = webots.wb_motor_get_force_feedback(tags.jointsByName.AnkleL)
      l_ft[5] = webots.wb_motor_get_force_feedback(tags.jointsByName.FootL)
      if tags.left_ankle_yaw > 0 then
        l_ft[6] = webots.wb_motor_get_force_feedback(tags.left_ankle_yaw)
      end
			dcm.set_sensor_lfoot(l_ft)
      --
      local r_ft = Body.get_rfoot()
			r_ft[2], r_ft[3], r_ft[1] = unpack(webots.wb_touch_sensor_get_values(tags.r_ft))
      r_ft[5] = webots.wb_motor_get_force_feedback(tags.jointsByName.AnkleR)
      r_ft[4] = webots.wb_motor_get_force_feedback(tags.jointsByName.FootR)
      if tags.right_ankle_yaw > 0 then
        r_ft[6] = webots.wb_motor_get_force_feedback(tags.right_ankle_yaw)
      end
			dcm.set_sensor_rfoot(r_ft)
    end

    -- GPS and compass data
    -- Webots x is our y, Webots y is our z, Webots z is our x,
    -- Our x is Webots z, Our y is Webots x, Our z is Webots y
    if ENABLE_POSE then
      local gps     = webots.wb_gps_get_values(tags.gps)
      local compass = webots.wb_compass_get_values(tags.compass)

			--local angle   = math.atan2( compass[3], compass[1] )
      --local pose    = vector.pose{gps[3], gps[1], angle}

			-- Fixed for robocup wbt
      local angle   = math.atan2( compass[1], -compass[3] )
      local pose    = vector.pose{-gps[1], gps[3], angle}

      --wcm.set_robot_pose( pose )
      wcm.set_robot_pose_gps( pose )
      local rpy = webots.wb_inertial_unit_get_roll_pitch_yaw(tags.inertialunit)

      --SJ: we need to remap rpy for webots
      dcm.sensorPtr.rpy[0], dcm.sensorPtr.rpy[1], dcm.sensorPtr.rpy[2] =
        rpy[2], rpy[1], -rpy[3]

      --[[
      print('rpy',unpack(rpy) )
      print('gps',unpack(gps) )
      print('compass',unpack(compass) )
      print('pose', pose )
      print()
      --]]
    end

		-- Update the sensor readings of the joint positions
		local rad, val
		local positions = dcm.get_sensor_position()
    for idx, jtag in ipairs(tags.joints) do
      if jtag>0 then
				val = get_pos(jtag)
        rad = servo.direction[idx] * val - servo.rad_offset[idx]
				rad = rad==rad and rad or 0
				positions[idx] = rad
      end
    end
		dcm.set_sensor_position(positions)

    -- Grab a camera frame
    if ENABLE_CAMERA and t >= NEXT_CAMERA then
      local w = webots.wb_camera_get_width(tags.head_camera)
      local h = webots.wb_camera_get_height(tags.head_camera)
      local img = ImageProc.rgb_to_yuyv(webots.to_rgb(tags.head_camera), w, h)
			WebotsBody.update_head_camera(img, 2*w*h, 0, t)
			NEXT_CAMERA = get_time() + camera_timeStep / 1000
    end
    -- Grab a lidar scan
    if ENABLE_CHEST_LIDAR and t >= NEXT_CHEST_LIDAR then
      local n = webots.wb_camera_get_width(tags.chest_lidar)
			local fov = webots.wb_camera_get_fov(tags.chest_lidar)
			local res = fov / n
      local ranges = webots.wb_camera_get_range_image(tags.chest_lidar)
			local metadata = {
        n=n,res=res,t=t,angle=Body.get_lidar_position(),rpy=Body.get_rpy()
      }
			WebotsBody.update_chest_lidar(metadata,ranges)
      --local lidar_array = require'carray'.float(ranges, w)
			NEXT_CHEST_LIDAR = get_time() + lidar_timeStep / 1000
    end
    -- Grab a lidar scan
    if ENABLE_HEAD_LIDAR and t >= NEXT_HEAD_LIDAR then
      local n = webots.wb_camera_get_width(tags.head_lidar)
      local fov = webots.wb_camera_get_fov(tags.head_lidar)
      local res = fov / n
      local ranges = webots.wb_camera_get_range_image(tags.head_lidar)
      local metadata = {n=n,res=res,t=t,angle=Body.get_lidar_position()}
      WebotsBody.update_head_lidar(metadata,ranges)
      NEXT_HEAD_LIDAR = get_time() + lidar_timeStep / 1000
    end

		-- Receive webot messaging
		while webots.wb_receiver_get_queue_length(tags.receiver) > 0 do
	    -- get first message on the queue
	    ndata = webots.wb_receiver_get_data_size(tags.receiver)
	    msg = webots.wb_receiver_get_data(tags.receiver)
	    if #msg==14 then 
	    	local ball_gpsx=(tonumber(string.sub(msg,2,6))-5)*2
	    	local ball_gpsy=(tonumber(string.sub(msg,8,12))-5)*2
	    	wcm.set_robot_gpsball({ball_gpsx,ball_gpsy});
        local timestarted = wcm.get_robot_timestarted()
        if Config.stop_after_score and timestarted~=0 and ball_gpsx>4.5 then
          print("=========================================")
          print("=========================================")
          local score_time = get_time()-timestarted
          local score_min = math.floor(score_time/60)
          local score_sec = score_time%60          
          print(string.format(
            "SCORED AT %d min %d sec",score_min, score_sec))
          print("=========================================")
          print("=========================================")
          wcm.set_robot_timestarted(0)
          gcm.set_game_state(4) --Set to finished
          head_ch:send'teleop'
        end
--	    	print("ball:",ball_gpsx,ball_gpsy)
	  	elseif #msg==16 then     		
    		local obsx=(tonumber(string.sub(msg,2,6))-5)*2
    		local obsy=(tonumber(string.sub(msg,8,12))-5)*2
    		local obsid = tonumber(string.sub(msg,14,14))
--    		print("obs:",obsid,obsx,obsy)
				if obsid==1 then wcm.set_robot_gpsobs1({obsx,obsy})
				else wcm.set_robot_gpsobs2({obsx,obsy}) end
    	end	    
	    webots.wb_receiver_next_packet(tags.receiver)
	  end
		
		-- Grab keyboard input, for modifying items
    local key_code = webots.wb_robot_keyboard_get_key()
		if WebotsBody.USING_KB then
			WebotsBody.update(key_code)
		else
			WebotsBody.update()
			local key_char_lower = string.char(key_code):lower()
			local key_toggle = key_action[key_char_lower]
			if key_toggle and t-t_last_keypress>1 then
				key_toggle()
				t_last_keypress = t
			end
		end
	
	end -- update

	Body.exit = function()
	end
	Body.tags = tags

  --Touchdown check for webots
  --Use FSR sensors

  local FSR_threshold = 200
  Body.get_lfoot_touched = function()
    local LFSR = dcm.get-sensor_lfoot()
    if (LFSR[1]+LFSR[2]+LFSR[3]+LFSR[4])>FSR_threshold then
      return true
    else
      return false
    end
  end
  Body.get_rfoot_touched = function()
    local RFSR = dcm.get-sensor_rfoot()
    if (RFSR[1]+RFSR[2]+RFSR[3]+RFSR[4])>FSR_threshold then
      return true
    else
      return false
    end
  end
else -- webots check
  --Touchdown check for actual robot
  --Force torque sensor based
  Body.get_lfoot_touched = function()
    return false
  end
  Body.get_rfoot_touched = function()
    return false
  end
end

-- Exports for use in other functions
Body.get_time = get_time
Body.nJoint = nJoint
Body.jointNames = jointNames
Body.parts = Config.parts
Body.Kinematics = Kinematics

----------------------
-- Add the gripper API
----------------------
local lgrip1_id, lgrip2_id = unpack(Config.parts.LGrip)
local rgrip1_id, rgrip2_id = unpack(Config.parts.RGrip)
local lgrip_ids = {[lgrip1_id] = true, [lgrip2_id] = true}
local rgrip_ids = {[rgrip1_id] = true, [rgrip2_id] = true}
function Body.set_lgrip_mode(mode)
	local msg = {wr_reg='torque_mode', ids=lgrip_ids}
	if mode=='torque' then
		msg.val = {[lgrip1_id] = 1, [lgrip2_id] = 1}
	elseif mode=='position' then
		msg.val = {[lgrip1_id] = 0, [lgrip2_id] = 0}
	end
	dcm_ch:send(mpack(msg))
end
function Body.set_rgrip_mode(mode)
	local msg = {wr_reg='torque_mode', ids=rgrip_ids}
	if mode=='torque' then
		msg.val = {[rgrip1_id] = 1, [rgrip2_id] = 1}
	elseif mode=='position' then
		msg.val = {[rgrip1_id] = 0, [rgrip2_id] = 0}
	end
	dcm_ch:send(mpack(msg))
end
----------------------

-- Check the error from a desired transform tr
-- to a forwards kinematics of in IK solution q
local function check_ik_error( tr, tr_check, pos_tol, ang_tol )

  -- Tolerate a 1mm error in distance
  pos_tol = pos_tol or 0.001
  ang_tol = ang_tol or 0.1*DEG_TO_RAD

	local position_error = math.sqrt(
	( tr_check[1]-tr[1] )^2 +
	( tr_check[2]-tr[2] )^2 +
	( tr_check[3]-tr[3] )^2 )

	local angle_error = math.sqrt(
	util.mod_angle( tr_check[4]-tr[4] )^2 +
	util.mod_angle( tr_check[5]-tr[5] )^2 +
	util.mod_angle( tr_check[6]-tr[6] )^2 )

	-- If within tolerance, return true
  local in_tolerance = true
	if position_error>pos_tol then in_tolerance=false end
  if angle_error>ang_tol then in_tolerance=false end

--  if not in_tolerance then
if false then
    print("IK ERROR")
    print(string.format("tr0:%.2f %.2f %.2f %.2f %.2f %.2f tr:%.2f %.2f %.2f %.2f %.2f %.2f",
    tr_check[1],
    tr_check[2],
    tr_check[3],
    tr_check[4]*RAD_TO_DEG,
    tr_check[5]*RAD_TO_DEG,
    tr_check[6]*RAD_TO_DEG,
    tr[1],
    tr[2],
    tr[3],
    tr[4]*RAD_TO_DEG,
    tr[5]*RAD_TO_DEG,
    tr[6]*RAD_TO_DEG
    ))
    print(string.format("LArm: %.1f %.1f %.1f %.1f %.1f %.1f %.1f",unpack(
      vector.new(Body.get_larm_command_position())*RAD_TO_DEG     ) ))
    print(string.format("RArm: %.1f %.1f %.1f %.1f %.1f %.1f %.1f",unpack(
      vector.new(Body.get_rarm_command_position())*RAD_TO_DEG     ) ))
    print()
--    print(string.format("perr:%.4f aerr:%.2f",position_error, angle_error*Body.RAD_TO_DEG))
  end
	return in_tolerance
end


--TODO:fix here
--[[
-- will we ever use lower-dof arm for anything?
local nJointLArm = 7
local nJointRArm = 7
--where's servo.min_rad defined now?
-- It is in Config.servo, Config_THOROP_Robot.lua
--]]

local function check_larm_bounds(qL)
  --SJ: now we don't hacve nJointLArm definition
--[[  
  print("check larm bound, nJointLArm:",nJointLArm)
  for i=1,nJointLArm do
    if qL[i]<servo.min_rad[indexLArm+i-1] or qL[i]>servo.max_rad[indexLArm+i-1] then
--      print("out of range",i,"at ",qL_target[i]*RAD_TO_DEG)
      return false
    end
  end
  --]]
  return true  
end

local function check_rarm_bounds(qR)
  --[[
  for i=1,nJointRArm do
    if qR[i]<servo.min_rad[indexRArm+i-1] or qR[i]>servo.max_rad[indexRArm+i-1] then
--      print("out of range",i,"at ",qR_target[i]*RAD_TO_DEG)
      return false
    end
  end
  --]]
  return true
end

--]]

--SJ: Now we consider waist angle and bodyTilt into FK/IK calculation
--Which is read from SHM


-- Take in joint angles and output an {x,y,z,r,p,yaw} table
-- SJ: Now separated into two functions to get rid of directly calling IK
Body.get_forward_larm = function(qL, bodyTilt, qWaist)
  local pLArm = Kinematics.l_arm_torso_7( qL,
    bodyTilt or mcm.get_stance_bodyTilt(),
    qWaist or Body.get_waist_command_position(),
    mcm.get_arm_lhandoffset()[1],mcm.get_arm_lhandoffset()[2],
    mcm.get_arm_lhandoffset()[3]
    )
  return pLArm
end

Body.get_forward_rarm = function(qR, bodyTilt, qWaist)
  local pRArm = Kinematics.r_arm_torso_7( qR,
    bodyTilt or mcm.get_stance_bodyTilt(),
    qWaist or Body.get_waist_command_position(),
    mcm.get_arm_rhandoffset()[1],mcm.get_arm_rhandoffset()[2],
    mcm.get_arm_rhandoffset()[3]
    )
  return pRArm
end

--Return the WRIST position (to test self collision)
Body.get_forward_lwrist = function(qL, bodyTilt, qWaist)
  local pLArm = Kinematics.l_wrist_torso( qL,
      bodyTilt or mcm.get_stance_bodyTilt(),
      qWaist or Body.get_waist_command_position())
  return pLArm
end

Body.get_forward_rwrist = function(qR, bodyTilt, qWaist)
  local pRArm = Kinematics.r_wrist_torso( qR,
      bodyTilt or mcm.get_stance_bodyTilt(),
      qWaist or Body.get_waist_command_position())
  return pRArm
end

Body.get_inverse_rwrist = function( qR, trR, rShoulderYaw, bodyTilt, qWaist)
  local qR_target = Kinematics.inverse_r_wrist(trR, qR,rShoulderYaw or qR[3],
      bodyTilt or mcm.get_stance_bodyTilt(), qWaist or Body.get_waist_command_position())
  return qR_target
end

Body.get_inverse_lwrist = function( qL, trL, lShoulderYaw, bodyTilt, qWaist)
  local qL_target = Kinematics.inverse_l_wrist(trL, qL, lShoulderYaw or qL[3],
      bodyTilt or mcm.get_stance_bodyTilt(), qWaist or Body.get_waist_command_position())
  return qL_target
end

Body.get_inverse_arm_given_wrist = function( q, tr, bodyTilt, qWaist)
  local q_target = Kinematics.inverse_arm_given_wrist(tr,q,
    bodyTilt or mcm.get_stance_bodyTilt(), qWaist or Body.get_waist_command_position())
  return q_target
end


Body.get_inverse_larm = function( qL, trL, lShoulderYaw, bodyTilt, qWaist)
  local shoulder_flipped = 0
  if qL[2]>math.pi/2 then shoulder_flipped=1 end

  local qL_target = Kinematics.inverse_l_arm_7(
    trL,qL,
    lShoulderYaw or qL[3],
    bodyTilt or mcm.get_stance_bodyTilt(),
    qWaist or Body.get_waist_command_position(),
    mcm.get_arm_lhandoffset()[1],
    mcm.get_arm_lhandoffset()[2],
    mcm.get_arm_lhandoffset()[3],
    shoulder_flipped
    )

  local trL_check = Kinematics.l_arm_torso_7(
    qL_target,
    bodyTilt or mcm.get_stance_bodyTilt(),
    qWaist or Body.get_waist_command_position(),
    mcm.get_arm_lhandoffset()[1],
    mcm.get_arm_lhandoffset()[2],
    mcm.get_arm_lhandoffset()[3]
    )

  local passed = check_larm_bounds(qL_target) and check_ik_error( trL, trL_check)
  if passed then return qL_target end
end
--
Body.get_inverse_rarm = function( qR, trR, rShoulderYaw, bodyTilt, qWaist)
  local shoulder_flipped = 0
  if qR[2]<-math.pi/2 then shoulder_flipped=1 end
  local qR_target = Kinematics.inverse_r_arm_7(
    trR, qR,
    rShoulderYaw or qR[3],
    bodyTilt or mcm.get_stance_bodyTilt(),
    qWaist or Body.get_waist_command_position(),
    mcm.get_arm_rhandoffset()[1],
    mcm.get_arm_rhandoffset()[2],
    mcm.get_arm_rhandoffset()[3],
    shoulder_flipped
    )

  local trR_check = Kinematics.r_arm_torso_7(
    qR_target,
    bodyTilt or mcm.get_stance_bodyTilt(),
    qWaist or Body.get_waist_command_position(),
    mcm.get_arm_rhandoffset()[1],
    mcm.get_arm_rhandoffset()[2],
    mcm.get_arm_rhandoffset()[3]
    )

  local passed = check_rarm_bounds(qR_target) and check_ik_error( trR, trR_check)
  if passed then return qR_target end
end
--

---------------------------------------------
-- New hand API
-- Positive force value for closing
-- Negative force value for openning
---------------------------------------------


--Used only for webots

if IS_WEBOTS then
  Body.finger_target={0,0,0,0}
  Body.finger_pos={0,0,0,0}

  Body.control_finger= function(finger_index,force)
    if force>0 then
      Body.finger_target[finger_index] = 1 --close
    elseif force<0 then
      Body.finger_target[finger_index] = 0 --open
    end
  end
  Body.update_finger = function(dt)
    Body.finger_pos = util.approachTol(
      Body.finger_pos,Body.finger_target,
      {2,2,2,2},dt)

    Body.set_lgrip_percent(Body.finger_pos[2])

    Body.set_rgrip_percent(Body.finger_pos[4])
  end

  Body.move_lgrip1 = function(force) Body.control_finger(1, force) end
  Body.move_lgrip2 = function(force) Body.control_finger(2, force) end
  Body.move_rgrip1 = function(force) Body.control_finger(3, force) end
  Body.move_rgrip2 = function(force) Body.control_finger(4, force) end

else
  Body.move_lgrip1 = Body.set_ltrigger_command_torque
  Body.move_lgrip2 = Body.set_lgrip_command_torque
  Body.move_rgrip1 = Body.set_rtrigger_command_torque
  Body.move_rgrip2 = Body.set_rgrip_command_torque
end




return Body
