-- 58.5cm up from the z=0 of the arm
-- 41.5cm back from the x=0 position

local Body = {}

require'wcm'
require'mcm'

local K = require'YouBotKinematics'
Body.Kinematics = K
local si = require'simple_ipc'
local util = require'util'
local vector = require'vector'

-- Five degree of freedom arm
local nJoint = Config.nJoint

-- Table of servo properties
local servo = Config.servo

-- Hold our sensors and commands locally
-- Only use the shared memory API sparingly
local actuator, sensor = {}, {}
for i,v in ipairs(Config.sensors) do sensor[v] = vector.zeros(nJoint) end
for i,v in ipairs(Config.actuators) do actuator[v] = vector.zeros(nJoint) end
-- Add the gripper
actuator.gripper = 0
sensor.gripper = 0
-- Add the base
actuator.base = vector.zeros(3)
sensor.base = vector.zeros(3)

-- Commanding the arm position
-- YouBot can have two arms, so we call a single arm 'left' for now...
function Body.set_larm_command_position (val)
	assert(type(val)=='table' and #val==nJoint, 'Bad set_command!')
	for idx,v in ipairs(val) do actuator.command_position[idx] = v end
end
Body.get_larm_command_position = function ()
	return actuator.command_position
end
Body.get_larm_position = function ()
	return sensor.position
end
-- Right arm does nothing atm
function Body.set_rarm_command_position ()
end
function Body.get_rarm_command_position ()
end
function Body.get_rarm_position ()
end

-- Other API wrappers
Body.set_walk_velocity = function(vx, vy, va)
	mcm.set_walk_velocity{vx, vy, va}
end

if not IS_WEBOTS then
	require'dcm'
	local get_time = require'unix'.time
	Body.get_time = get_time
	local youbot

	-- Entry initializes the hardware of the robot
	function Body.entry ()
		youbot = require'youbot'
		youbot.init_base()
		youbot.init_arm()
		youbot.calibrate_gripper()

	  -- Set the initial joint command angles, so we have no jerk initially
		local pos, rad
	  for i=1,nJoint do
			local lower, upper, en = youbot.get_arm_joint_limit(i)
			pos = youbot.get_arm_position(i)
	    rad = (pos - servo.offset[i]) * servo.direction[i]
			sensor.position[idx] = rad
			rad = math.max(servo.min_rad[i], math.min(servo.max_rad[i], v))
			actuator.position[idx] = rad
	  end
	  mcm.set_walk_vel(vector.zeros(3))
	end

	function Body.update ()
	  -- Get joint readings
	  local rad, mps, nm = {},{},{}
	  for i=1,nJoint do
	    rad[i] = (youbot.get_arm_position(i) - servo.offset[i]) * servo.direction[i]
	    mps[i] = youbot.get_arm_velocity(i)
	    nm[i]  = youbot.get_arm_torque(i)
	  end
	  -- Set shm
	  dcm.set_sensor_position(rad)
	  dcm.set_sensor_velocity(mps)
	  dcm.set_sensor_torque(nm)

	  -- Set the gripper from shared memory
	  local spacing = dcm.get_actuator_command_gripper()
		if spacing~=gripper_pos then
			youbot.set_gripper_spacing(
				math.max(math.min(spacing,0.023),0)
			)
			-- Keep a local copy
			gripper_pos = spacing
		else
			-- Set joints from shared memory when not using the gripper
			local desired_pos = dcm.get_actuator_command_position()
			for i,v in ipairs(desired_pos) do
				-- Correct the direction and the offset
				local val = v * servo.direction[i] + servo.offset[i]
				-- Set the youbot arm
				youbot.set_arm_angle(i,val)
			end
		end

	  -- Set base from shared memory
	  local vel = mcm.get_walk_vel()
	  youbot.set_base_velocity( unpack(vel) )

		-- Get Odometry measurements
		local dx, dy, da = youbot.get_base_position()
		wcm.set_robot_odometry{dx,dy,da}

	end

	-- Exit gracefully shuts down the hardware of the robot
	function Body.exit ()
	  youbot.shutdown_arm()
	  youbot.shutdown_base()
	end

-- Webots overrides
else

  -- Default configuration (toggle during run time)
  local ENABLE_CAMERA = false
  local ENABLE_LIDAR  = false
  local ENABLE_KINECT = false
	local ENABLE_POSE   = true

	-- New modules
	local webots = require'webots'
	assert(webots.wb_motor_set_position, 'BAD WEBOTS 7 MODULE')
	local mp = require'msgpack.MessagePack'
	local lidar0_ch = si.new_publisher'lidar0'

  -- Start the system
  webots.wb_robot_init()
  -- Acquire the timesteps
  local timeStep = webots.wb_robot_get_basic_time_step()
  local camera_timeStep = math.max(33, timeStep)
  local lidar_timeStep = math.max(25, timeStep)

	-- Body time
  local get_time = webots.wb_robot_get_time
	Body.get_time = get_time

  -- Set the correct servo properties for Webots
  servo.direction = vector.new({1, -1, -1, -1, 1})
  assert(#servo.direction==nJoint,'Bad servo direction!')
	-- Offsets are zero for Webots
  servo.offset = servo.offset * 0

  -- Setup the webots tags
  local tags = {
		joints = {},
		gripper = {},
		wheels = {},
	}

  -- Ability to turn on/off items
  local t_last_keypress = get_time()
  -- Enable the keyboard 100ms
  webots.wb_robot_keyboard_enable(100)
  local key_action = {
    l = function(override)
			if override~=nil then en=override else en=ENABLE_LIDAR==false end
      if en==false then
        print(util.color('LIDAR disabled!','yellow'))
        webots.wb_camera_disable(tags.lidar)
        ENABLE_LIDAR = false
      else
        print(util.color('LIDAR enabled!','green'))
        webots.wb_camera_enable(tags.lidar,lidar_timeStep)
        ENABLE_LIDAR = true
      end
    end,
    c = function(override)
      if override~=nil then en=override else en=ENABLE_CAMERA==false end
      if en==false then
        print(util.color('CAMERA disabled!','yellow'))
        webots.wb_camera_disable(tags.hand_camera)
        ENABLE_CAMERA = false
      else
        print(util.color('CAMERA enabled!','green'))
        webots.wb_camera_enable(tags.hand_camera,camera_timeStep)
        ENABLE_CAMERA = true
      end
    end,
    k = function(override)
      if override~=nil then en=override else en=ENABLE_KINECT==false end
      if en==false then
        print(util.color('KINECT disabled!','yellow'))
        webots.wb_camera_disable(tags.kinect)
        ENABLE_KINECT = false
      else
        print(util.color('KINECT enabled!','green'))
        webots.wb_camera_enable(tags.kinect,camera_timeStep)
        ENABLE_KINECT = true
      end
    end,
		p = function(override)
			if override~=nil then en=override else en=ENABLE_POSE==false end
      if en==false then
        print(util.color('POSE disabled!','yellow'))
        webots.wb_gps_disable(tags.gps)
	  		webots.wb_compass_disable(tags.compass)
        ENABLE_POSE = false
      else
        print(util.color('POSE enabled!','green'))
        webots.wb_gps_enable(tags.gps, timeStep)
	  		webots.wb_compass_enable(tags.compass, timeStep)
        ENABLE_POSE = true
      end
    end,
  }

	local function get_pose ()
		local gps     = webots.wb_gps_get_values(tags.gps)
		local compass = webots.wb_compass_get_values(tags.compass)
		local angle   = math.atan2(compass[3], compass[1])
		return vector.pose{gps[3], gps[1], util.mod_angle(angle + 90 * DEG_TO_RAD)}
	end

	local neg_inf, pos_inf = -1 / 0, 1 / 0
	local function wb_wheel_speed (tag, speed)
		if speed > 0 then
			webots.wb_motor_set_position(tag,pos_inf)
		else
			webots.wb_motor_set_position(tag,neg_inf)
		end
		webots.wb_motor_set_velocity(tag,math.abs(speed))
	end

	local function wheel_helper (vx, vy, va)
		local K1,K2,K3 = 10,10,10
		local v1,v2,v3,v4 = 0,0,0,0
		-- First, the angle
		v1 = v1 + va * K1
		v2 = v2 - va * K1
		v3 = v3 + va * K1
		v4 = v4 - va * K1
		-- Second, forward
		v1 = v1 + vx * K2
		v2 = v2 + vx * K2
		v3 = v3 + vx * K2
		v4 = v4 + vx * K2
		-- Third, strafe
		v1 = v1 + vy * K3
		v2 = v2 - vy * K3
		v3 = v3 - vy * K3
		v4 = v4 + vy * K3
		-- Set the speeds
		wb_wheel_speed(tags.wheels[1],v1)
		wb_wheel_speed(tags.wheels[2],v2)
		wb_wheel_speed(tags.wheels[3],v3)
		wb_wheel_speed(tags.wheels[4],v4)
	end

  function Body.entry ()

    -- Grab the joints
  	for i=1,nJoint do
  		tags.joints[i] = webots.wb_robot_get_device('arm'..i)
  		assert(tags.joints[i]>0, 'Arm '..i..' not found')
      webots.wb_motor_enable_position(tags.joints[i], timeStep)
			webots.wb_motor_set_position(tags.joints[i], 0)
  	end
    -- Grab the wheels
  	for i=1,4 do
  		tags.wheels[i] = webots.wb_robot_get_device('wheel'..i)
			assert(tags.wheels[i]>0, 'Wheel '..i..' not found')
  	end
    -- Grab the fingers
  	for i=1,2 do
  		tags.gripper[i] = webots.wb_robot_get_device('finger'..i)
			assert(tags.gripper[i]>0, 'Finger '..i..' not found')
      --webots.wb_motor_set_velocity(tags.gripper[i], 0.03)
  	end
    -- Acquire sensor tags
		tags.gps = webots.wb_robot_get_device("GPS")
		tags.compass = webots.wb_robot_get_device("compass")
		tags.hand_camera = webots.wb_robot_get_device("HandCamera")
		tags.lidar = webots.wb_robot_get_device("lidar")
		tags.kinect = webots.wb_robot_get_device("kinect")

		-- Enable sensors
		key_action.p(ENABLE_POSE)
		key_action.c(ENABLE_CAMERA)
		key_action.l(ENABLE_LIDAR)
		key_action.k(ENABLE_KINECT)

		-- Step the simulation
		webots.wb_robot_step(timeStep)

		-- Form and set the pose
		wcm.set_robot_initialpose(get_pose())

		-- Step it again
    webots.wb_robot_step(timeStep)

    -- Read values
		local pos, rad
    for idx, jtag in pairs(tags.joints) do
      pos = webots.wb_motor_get_position( jtag )
      -- Take care of nan
      if pos~=pos then pos = 0 end
			rad = (pos - servo.offset[idx]) * servo.direction[idx]
			sensor.position[idx] = rad
			actuator.command_position[idx] = math.max(servo.min_rad[idx], math.min(servo.max_rad[idx], rad))
    end
		-- Zero the base velocity
		actuator.base = actuator.base * 0
		sensor.base = sensor.base * 0
  end

	-- Simple vision routine for webots
	--[[
	local lV = require'libVision'
	local meta_b = {
		id = 'labelB',
		w = w / sA / sB,
		h = h / sA / sB,
		c = 'zlib',
	}
	local s_t = simple_ipc.new_publisher('top')
	local zlib = require'zlib.ffi'
	local c_zlib = zlib.compress_cdata
	--]]
	local function update_vision ()
		if not ENABLE_CAMERA then return end
		local w = webots.wb_camera_get_width(tags.hand_camera)
		local h = webots.wb_camera_get_height(tags.hand_camera)
		local yuyv = ImageProc.rgb_to_yuyv(webots.to_rgb(tags.hand_camera), w, h)
		lV.entry(Config.vision[1])
		local dbg_top = lV.update(im_top)
		-- Send all the debug information
		s_t:send(top_debug)
	end

	local function update_kinect ()
		if not ENABLE_KINECT then return end
		local w = webots.wb_camera_get_width(tags.kinect)
		local h = webots.wb_camera_get_height(tags.kinect)
		local color_fr = webots.to_rgb(tags.kinect)
		local depth_fr = webots.wb_camera_get_range_image(tags.kinect)
		local sz = w*h
		local depth_array = carray.float( depth_fr, sz )
		if not depth_torch then
			-- Perform the initial allocation
			depth_torch = torch.FloatTensor( sz ):zero()
			depth_byte  = torch.ByteTensor( sz ):zero()
			depth_adj   = torch.FloatTensor( sz ):zero()
		end
		depth_array:tensor(depth_torch)
		local near, far = .1, 2
		-- Enhance the dynamic range of the mesh image
		depth_adj:copy(depth_torch):add( -near )
		depth_adj:mul( 255/(far-near) )
		-- Ensure that we are between 0 and 255
		depth_adj[torch.lt(depth_adj,0)] = 0
		depth_adj[torch.gt(depth_adj,255)] = 255
		depth_byte:copy( depth_adj )
		-- Compress images
		local depth_jpg = jpeg.compress_gray( depth_byte:storage():pointer(), w, h )
		local color_jpg = jpeg.compress_rgb(color_fr,w,h)
	end

	local function update_lidar ()
		-- Grab a lidar scan
		if not ENABLE_LIDAR then return end
		local w = webots.wb_camera_get_width(tags.lidar)
		local lidar_fr = webots.wb_camera_get_range_image(tags.lidar)
		local lidar_array = carray.float( lidar_fr, w )
		-- Send the message on the lidar channel
		local meta = {
			t     = get_time(),
			n     = w,
			res   = 360 / 1440,
			pose  = get_pose(),
			angle = 0, -- no actuation
		}
		lidar0_ch:send{mp.pack(meta),tostring(lidar_array)}
	end

  function Body.update ()
		local t = get_time()
    -- Write arm commands
		local rad
		for idx, jtag in ipairs(tags.joints) do
			rad = actuator.command_position[idx] * servo.direction[idx] + servo.offset[idx]
      webots.wb_motor_set_position(jtag, math.max(servo.min_rad[idx], math.min(servo.max_rad[idx], rad)))
    end

    -- Base
    local vel = mcm.get_walk_vel()
    wheel_helper(unpack(vel))

    -- Gripper
    --local spacing = dcm.get_actuator_command_gripper()
		local spacing = 0
    local width = math.max(math.min(spacing, 0.025), 0)
    webots.wb_motor_set_position(tags.gripper[1], width)
    webots.wb_motor_set_position(tags.gripper[2], width)

		-- Step the simulation, and shutdown if the update fails
		if webots.wb_robot_step(timeStep) < 0 then
			print('EPIC FAIL UPDATE')
			os.exit()
		end

    -- Read values
    for idx, jtag in ipairs(tags.joints) do
			sensor.position[idx] = (webots.wb_motor_get_position(jtag) - servo.offset[idx]) * servo.direction[idx]
    end

    -- Get sensors
		if ENABLE_POSE then wcm.set_robot_gps(get_pose()) end
    -- Update our vision system
    update_vision()
		update_kinect()
    update_lidar()

    -- Grab keyboard input, for modifying items
    local key_code = webots.wb_robot_keyboard_get_key()
    local key_char = string.char(key_code)
    local key_char_lower = string.lower(key_char)
    local key_toggle = key_action[key_char_lower]
    if key_toggle and t-t_last_keypress>1 then
      key_toggle()
      t_last_keypress = t
    end

  end

  function Body.exit ()
  end

end

return Body
