--------------------------------
-- Body abstraction for THOR-OP
-- (c) 2013,2014 Stephen McGill, Seung-Joon Yi
--------------------------------
--assert(ffi, 'Need LuaJIT to run. Lua support in the future')
-- Shared memory for the joints
require'dcm'

-- Utilities
local vector = require'vector'
local util   = require'util'
local si     = require'simple_ipc'
local Kinematics = require'THOROPKinematics'
local mpack  = require'msgpack'.pack

local Body = {}
local dcm_ch, imu_ch, imu_thread, dcm_thread
-- Default is not in thread for dcm_ch
local dcm_ch = si.new_publisher'dcm!'
local get_time
if not unix then
	get_time = require'unix'.time
else
	get_time = unix.time
end
local vslice = vector.slice

------------------
-- Body sensors --
------------------
local nx_registers = require'libDynamixel'.nx_registers
for _, sensor in ipairs(dcm.sensorKeys) do
	local cur = dcm['get_sensor_'..sensor]()
	local n_el = type(cur)=='table' and #cur or 1
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

--------------------
-- Body actuators --
--------------------
--for actuator, ptr in pairs(dcm.actuatorPtr) do
for _, actuator in ipairs(dcm.actuatorKeys) do
	local cur = dcm['get_actuator_'..actuator]()
	local n_el = type(cur)=='table' and #cur or 1
	-- Only command_position is constantly synced
	-- Other commands need to be specially sent to the Body
	local not_synced = actuator~='command_position'
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

-- If receiving data from a chain
local function dcm_cb()
  dcm_ch:receive()
end
local function imu_cb()
  local msgs = imu_ch:receive()
  for _, msg in ipairs(msgs) do print(msg) end
end

function Body.entry()
	-- DCM
	dcm_ch, dcm_thread =
	  si.new_thread(ROBOT_HOME..'/run_dcm.lua', 'dcm')
	ch.callback = dcm_cb
	-- IMU
	imu_ch, imu_thread =
		si.new_thread(ROBOT_HOME..'/run_imu.lua', 'imu', v)
	imu_ch.callback = imu_cb
	-- Polling object
	body_poll = si.wait_on_channels({dcm_ch, imu_ch})
  -- Start the threads
  dcm_thread:start()
	imu_thread:start()
end

function Body.update()
	-- Poll for events
	-- Return immediately if nothing happening
	-- NOTE: Most of the time, nothing will happen...
	body_poll:poll()
end

function Body.exit()
	-- Tell the devices to exit cleanly
  if imu_thread then
    imu_ch:send'exit'
    imu_thread:join()
  end
  if dcm_thread then
    dcm_ch:send'exit'
    dcm_thread:join()
  end
end

---
-- Special functions
---
function Body.enable_read(chain)
print("EN READ")
  dcm_ch:send(mpack({bus=chain,key='enable_read', val=true}))
end
function Body.disable_read(chain)
print("DIS READ")
  dcm_ch:send(mpack({bus=chain,key='enable_read', val=false}))
end

----------------------
-- Webots compatibility
if IS_WEBOTS then
	local WebotsBody
	-- Shared memory for world
	require'wcm'
  
  Body.enable_read = function(chain)
  end
  Body.disable_read = function(chain)
  end

  --SJ: I put test_walk capabality here
  local fsm_chs = {}
  for _,sm in ipairs(Config.fsm.enabled) do
    local fsm_name = sm..'FSM'
    table.insert(fsm_chs, fsm_name)
    _G[sm:lower()..'_ch'] = si.new_publisher(fsm_name.."!")
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
  local ENABLE_CAMERA = true --false
  local ENABLE_LOG, t_log = false, 0
  if ENABLE_LOG then
  	libLog = require'libLog'
  	logger = libLog.new('yuyv', true)
  end
  
  require'hcm'

  --Added to config rather than hard-code 
  local ENABLE_CHEST_LIDAR  = Config.sensors.chest_lidar
  local ENABLE_HEAD_LIDAR = Config.sensors.head_lidar
  local ENABLE_FSR = Config.sensors.fsr
  local ENABLE_FT = Config.sensors.ft

  local ENABLE_KINECT = false
  local ENABLE_POSE   = true
  local ENABLE_IMU   = true

  local torch = require'torch'
  torch.Tensor = torch.DoubleTensor
  local carray = require'carray'
  local jpeg = require'jpeg'

	-- Publish sensor data
	local simple_ipc = require'simple_ipc'
	local mp = require'msgpack.MessagePack'
	local lidar0_ch = simple_ipc.new_publisher'lidar0'
	local lidar1_ch = simple_ipc.new_publisher'lidar1'

  local webots = require'webots'
	local carray = require'carray'
  -- Start the system
  webots.wb_robot_init()
  -- Acquire the timesteps
  local timeStep = webots.wb_robot_get_basic_time_step()
  local camera_timeStep = math.max(33,timeStep)
  local lidar_timeStep = math.max(25,timeStep)
  Body.timeStep = timeStep
  get_time = webots.wb_robot_get_time

  -- Setup the webots tags
  local tags = {}  
  local t_last_error = -math.huge

  tags.receiver = webots.wb_robot_get_device("receiver")
  webots.wb_receiver_enable(tags.receiver,timeStep)
  webots.wb_receiver_set_channel(tags.receiver,13)
    
	-- Vision routines
  local udp = require'udp'
  local jpeg = require'jpeg'
  local c_yuyv = jpeg.compressor('yuyv')
  -- Load vision config
	local cam_cfg = Config.camera[1]
  print('Color table loaded', cam_cfg.lut)
  local operator = Config.net.operator.wired
	local cam_udp_ch = udp.new_sender(operator, cam_cfg.udp_port)
	-- Just use one detection routine
  local vision = require(cam_cfg.detection_pipeline[1])
	vision.entry(cam_cfg, Body)
  local t_vision_send, SEND_VISION_INTERVAL = 0
	local function update_vision(yuyv)
		vision.update(yuyv)
    local t_now = Body.get_time()
    if t_now - t_vision_send > SEND_VISION_INTERVAL then
      t_vision_send = t_now
      local udp_data, udp_ret, udp_err 
  		for _,v in ipairs(vision.send()) do
  			if v[2] then
  				udp_data = mp.pack(v[1])..v[2]
  			else
  				udp_data = mp.pack(v[1])
  			end
  			udp_ret, udp_err = cam_udp_ch:send(udp_data)
      end
    end
	end

  -- Ability to turn on/off items
  local t_last_keypress = get_time()
  -- Enable the keyboard 100ms
  webots.wb_robot_keyboard_enable( 100 )
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
  }
	local OLD_API = false
	local set_pos, get_pos = webots.wb_motor_set_position, webots.wb_motor_get_position
	if OLD_API then
		set_pos = webots.wb_servo_set_position
		get_pos = webots.wb_servo_get_position
	end
	function Body.entry()
    -- Request @ t=0 to always be earlier than position reads

		-- Grab the tags from the joint names
		tags.joints = {}

		for i,v in ipairs(jointNames) do
      local tag = webots.wb_robot_get_device(v)
			if tag>0 then
				if OLD_API then
					webots.wb_servo_enable_position(tag, timeStep)
	        webots.wb_servo_set_velocity(tag, 4)
				else
					webots.wb_motor_enable_position(tag, timeStep)
					webots.wb_motor_set_velocity(tag, 4)
				end
        tags.joints[i] = tag
			end
		end

		-- Add Sensor Tags
		tags.accelerometer = webots.wb_robot_get_device("Accelerometer")
		tags.gyro = webots.wb_robot_get_device("Gyro")
		tags.gps = webots.wb_robot_get_device("GPS")
		tags.compass = webots.wb_robot_get_device("Compass")
		tags.inertialunit = webots.wb_robot_get_device("InertialUnit")

    if Config.sensors.head_lidar then
      tags.chest_lidar = webots.wb_robot_get_device("ChestLidar")
    end
    if Config.sensors.chest_lidar then
      tags.head_lidar = webots.wb_robot_get_device("HeadLidar")
    end
    if Config.sensors.fsr then
      tags.l_fsr = webots.wb_robot_get_device("L_FSR")
      tags.r_fsr = webots.wb_robot_get_device("R_FSR")
    end
    if Config.sensors.ft then

    end
		tags.head_camera = webots.wb_robot_get_device("HeadCamera")
    
		-- Enable or disable the sensors
		key_action.i(ENABLE_IMU)
		key_action.p(ENABLE_POSE)
		key_action.c(ENABLE_CAMERA)
		--key_action.h(ENABLE_HEAD_LIDAR)
		--key_action.l(ENABLE_CHEST_LIDAR)
		--key_action.k(ENABLE_KINECT)
		--key_action.f(ENABLE_FSR)

		-- Take a step to get some values
		webots.wb_robot_step(timeStep)

		local rad, val
		local positions = {}
    for idx, jtag in ipairs(tags.joints) do
      if jtag>0 then
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

    local t = Body.get_time()
    --Body.update_finger(timeStep)

		-- Set actuator commands from shared memory
		local cmds = Body.get_command_position()
		local poss = Body.get_position()
		for idx, jtag in ipairs(tags.joints) do
			local cmd, pos = cmds[idx], poss[idx]

			-- TODO: What is velocity?
			local vel = 0 or Body.get_command_velocity()[idx]
			local en  = 1 or Body.get_torque_enable()[idx]
			local deltaMax = timeStep * vel
			-- Only update the joint if the motor is torqued on

			-- If the joint is moving
			-- Clamp the difference between commanded and actuated
			-- so that we don't have huge jumped
			-- NOTE: This *should* be handled by the simulator?

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

			if en>0 and jtag>0 then
        local rad = servo.direction[idx] * (new_pos + servo.rad_offset[idx])
                
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
        local val = get_pos(jtag)
        local delta = util.mod_angle(rad-val)
        set_pos(jtag, rad+delta)
      end
		end --for

		-- Step the simulation, and shutdown if the update fails
		if webots.wb_robot_step(Body.timeStep) < 0 then os.exit() end

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
      dcm.sensorPtr.lfoot[0] = webots.wb_touch_sensor_get_value(tags.l_fsr)*4
      dcm.sensorPtr.rfoot[0] = webots.wb_touch_sensor_get_value(tags.r_fsr)*4
    end

    -- GPS and compass data
    -- Webots x is our y, Webots y is our z, Webots z is our x,
    -- Our x is Webots z, Our y is Webots x, Our z is Webots y
    if ENABLE_POSE then
      local gps     = webots.wb_gps_get_values(tags.gps)
      local compass = webots.wb_compass_get_values(tags.compass)

--SJ:fixed for robocup wbt
      --local angle   = math.atan2( compass[3], compass[1] )
      --local pose    = vector.pose{gps[3], gps[1], angle}
      local angle   = math.atan2( compass[1], -compass[3] )
      local pose    = vector.pose{-gps[1], gps[3], angle}

      --wcm.set_robot_pose( pose )
      wcm.set_robot_pose_gps( pose )
      local rpy = webots.wb_inertial_unit_get_roll_pitch_yaw(tags.inertialunit)

      --SJ: we need to remap rpy for webots
      dcm.sensorPtr.rpy[0],dcm.sensorPtr.rpy[1],dcm.sensorPtr.rpy[2] =
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
		-- TODO: If a joint is not found?
		local val, rad
		for idx, jtag in ipairs(tags.joints) do
			if jtag>0 then
				val = get_pos(jtag)
				if val~=val then val = 0 end
				rad = servo.direction[idx] * val - servo.rad_offset[idx]
        dcm.sensorPtr.position[idx-1] = rad
			end
		end
		--print('pos', dcm.get_sensor_position())
    --dcm.set_sensor_position(dcm.get_actuator_command_position())

    -- Grab a camera frame
    if ENABLE_CAMERA then
      local w = webots.wb_camera_get_width(tags.head_camera)
      local h = webots.wb_camera_get_height(tags.head_camera)
      local img = ImageProc.rgb_to_yuyv(webots.to_rgb(tags.head_camera), w, h)
      -- Logs for making colortable
      local meta = {
        t = t,
        sz = 0,
        w = w,
        h = h,
        id = 'head_camera',
        c = 'jpeg',
      }
      local t_now = Body.get_time()
      local LOG_INTERVAL = 1/20
      if logger and t_now - t_log> LOG_INTERVAL then
        meta.t = t_now
        --TODO: meta.cnt?
        local sz = 4/2*w*h  -- 4 bytes per 2 pixels
    		meta.rsz = sz
        meta[1] = vision.get_metadata()
        logger:record(meta, img, sz)
        t_log = t_now
      end

      -- Vision routines
      SEND_VISION_INTERVAL = 1 / hcm.get_monitor_fps()
      update_vision(img)
    end
    -- Grab a lidar scan
    if ENABLE_CHEST_LIDAR then
      local w = webots.wb_camera_get_width(tags.chest_lidar)
      local lidar_fr = webots.wb_camera_get_range_image(tags.chest_lidar)
      local lidar_array = carray.float( lidar_fr, w )
			-- Send the message on the lidar channel
			local meta = {}
			meta.t     = Body.get_time()
			meta.n     = w
			meta.res   = 360 / 1440
			meta.pose  = wcm.get_robot_pose()
			lidar0_ch:send{mp.pack(meta),tostring(lidar_array)}
    end
    -- Grab a lidar scan
    if ENABLE_HEAD_LIDAR then
      local w = webots.wb_camera_get_width(tags.head_lidar)
      local lidar_fr = webots.wb_camera_get_range_image(tags.head_lidar)
      local lidar_array = carray.float( lidar_fr, w )
			-- Send the message on the lidar channel
			local meta = {}
			meta.t     = Body.get_time()
			meta.n     = w
			meta.res   = 360 / 1440
			meta.pose  = wcm.get_robot_pose()
			lidar1_ch:send{mp.pack(meta),tostring(lidar_array)}
    end

    -- Grab keyboard input, for modifying items
    local key_code = webots.wb_robot_keyboard_get_key()
    local key_char = string.char(key_code)
    local key_char_lower = string.lower(key_char)

    if t-t_last_keypress>1 then
--[[      
      if key_char_lower=='1' then
        body_ch:send'init'
        head_ch:send'teleop'           
        t_last_keypress = t
--]]

      if key_char_lower=='1' then
        --Attacker start
        gcm.set_game_role(1)
        gcm.set_game_state(0)

        t_last_keypress = t

      elseif key_char_lower=='2' then

        --Goalie start
        gcm.set_game_role(0) --Zero: goalie
        gcm.set_game_state(0)
        t_last_keypress = t

      elseif key_char_lower=='3' then
        --Tester start
        gcm.set_game_role(2) --tester role
        gcm.set_game_state(6) --tester state
        t_last_keypress = t

      elseif key_char_lower=='4' then
        gcm.set_game_state(3) --Playing
        print("PLAYING")
        t_last_keypress = t

      elseif key_char_lower=='8' then        
        motion_ch:send'stand'
        if mcm.get_walk_ismoving()>0 then 
          print("requesting stop")
          mcm.set_walk_stoprequest(1) 
        end
        t_last_keypress = t        
	  elseif key_char_lower=='9' then        
        motion_ch:send'hybridwalk'
        t_last_keypress = t                
      elseif key_char_lower=='f' then
        --head_ch:send'scan'
        head_ch:send'scanobs'
        t_last_keypress = t
      elseif key_char_lower=='g' then
        body_ch:send'play'
        head_ch:send'scan'    
        t_last_keypress = t        
      end
    end

    --No need to toggle anything for robocup testing
--[[
		local key_toggle = key_action[key_char_lower]
    if key_toggle and t-t_last_keypress>1 then
      key_toggle()
      t_last_keypress = t
    end
--]]


	--Receive webot messaging
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
		
		WebotsBody.update()
	
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

local function check_larm_bounds(qL)
  for i=1,nJointLArm do
    if qL[i]<servo.min_rad[indexLArm+i-1] or qL[i]>servo.max_rad[indexLArm+i-1] then
--      print("out of range",i,"at ",qL_target[i]*RAD_TO_DEG)
      return false
    end
  end
  return true
end

local function check_rarm_bounds(qR)
  for i=1,nJointRArm do
    if qR[i]<servo.min_rad[indexRArm+i-1] or qR[i]>servo.max_rad[indexRArm+i-1] then
--      print("out of range",i,"at ",qR_target[i]*RAD_TO_DEG)
      return false
    end
  end
  return true
end

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
