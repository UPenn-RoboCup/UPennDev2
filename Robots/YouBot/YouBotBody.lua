-- 58.5cm up from the z=0 of the arm
-- 41.5cm back from the x=0 position

local Body = {}

-- Utilities
local unix       = require'unix'
local vector     = require'vector'
local util       = require'util'
local youbot
require'wcm'
require'jcm'
require'mcm'

-- Kinematics
local Transform = require'Transform'
local K = require'YouBotKinematics'
Body.Kinematics = K

-- Get time (for the real robot)
local get_time = unix.time

-- Store the gripper position
local gripper_pos

-- Five degree of freedom arm
local nJoint = 5
assert(nJoint==Config.nJoint,
	'Config file and Body must agree on nuber of joints!')

-- Table of servo properties
local servo = {}
-- Real Robot is the default
servo.min_rad = vector.new({
  -165,-57,-142,-95,-150,
})*DEG_TO_RAD
assert(#servo.min_rad==nJoint,'Bad servo min_rad!')
servo.max_rad = vector.new({
  165,89,142,95,150,
})*DEG_TO_RAD
servo.direction = vector.new({
  1,-1,1,-1,1
})
assert(#servo.direction==nJoint,'Bad servo direction!')
-- Offsets represent the actual zero position
servo.offset = vector.new({
  -167,-58,-143,-97,-168
})*DEG_TO_RAD
assert(#servo.offset==nJoint,'Bad servo offsets!')

-- Convienence function to get each joint sensor
for i,v in ipairs{'torque','velocity','position'} do
  Body['get_'..v] = jcm['get_sensor_'..v]
  Body['set_'..v] = jcm['set_sensor_'..v]
end

Body.set_command_position = function(val)
	assert(type(val)=='table' and #val==nJoint,'Bad set_command!')
	local clamped = util.clamp_vector(val,servo.min_rad,servo.max_rad)
	jcm.set_actuator_command_position(clamped)
end
Body.get_command_position = jcm.get_actuator_command_position

-- Base convience
Body.set_velocity = function(x,y,a)
  mcm.set_walk_vel{x,y,a}
end
Body.get_velocity = mcm.get_walk_vel

-- Entry initializes the hardware of the robot
Body.entry = function()
	youbot = require'youbot'
	youbot.init_base()
	youbot.init_arm()
	youbot.calibrate_gripper()

  -- Set the initial joint command angles, so we have no jerk initially
  local init_pos = {}
  for i=1,nJoint do
		local lower, upper, en = youbot.get_arm_joint_limit(i)
		local pos = youbot.get_arm_position(i)
    local rad = (pos - servo.offset[i]) * servo.direction[i]
    init_pos[i] = rad
  end
	Body.set_command_position(init_pos)
  jcm.set_sensor_position(init_pos)
  mcm.set_walk_vel(vector.zeros(3))
	return 1
end

-- Update speaks to the hardware of the robot
--Body.update_cycle = 0.050 -- seems ok
Body.update_cycle = 0.001
--Body.update_cycle = 0.2
Body.nop = function() end
-- cnt of 0 is a reset
Body.update = function(cnt)
  -- Get joint readings
  local rad,mps,nm = {},{},{}
  for i=1,nJoint do
		local pos = youbot.get_arm_position(i)
    rad[i] = (pos - servo.offset[i]) * servo.direction[i]
    mps[i] = youbot.get_arm_velocity(i)
    nm[i]  = youbot.get_arm_torque(i)
  end
  -- Set shm
  jcm.set_sensor_position(rad)
  jcm.set_sensor_velocity(mps)
  jcm.set_sensor_torque(nm)

  -- Set the gripper from shared memory
  local spacing = jcm.get_gripper_command_position()[1]
	if spacing~=gripper_pos then
		youbot.set_gripper_spacing(
			math.max(math.min(spacing,0.023),0)
		)
		-- Keep a local copy
		gripper_pos = spacing
	else
		-- Set joints from shared memory when not using the gripper
		local desired_pos = jcm.get_actuator_command_position()
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

	-- Increment the counter... why?
	if cnt then return cnt+1 end

end

-- Exit gracefully shuts down the hardware of the robot
Body.exit = function()
  youbot.shutdown_arm()
  youbot.shutdown_base()
end

-- Webots overrides
if IS_WEBOTS then

  -- Default configuration (toggle during run time)
  local ENABLE_CAMERA = false
  local ENABLE_LIDAR  = false
  local ENABLE_KINECT = false
	local ENABLE_POSE   = true

  local torch = require'torch'
  torch.Tensor = torch.DoubleTensor
  local carray = require'carray'
  local jpeg = require'jpeg'
	local c_rgb = jpeg.compressor('rgb')

	-- Publish sensor data
	local simple_ipc = require'simple_ipc'
	local mp = require'msgpack'
	local lidar0_ch = simple_ipc.new_publisher'lidar0'
	local camera0_ch = simple_ipc.new_publisher'camera0'

  local webots = require'webots'
	assert(webots.wb_motor_set_position, 'BAD WEBOTS MODULE')
  -- Start the system
  webots.wb_robot_init()
  -- Acquire the timesteps
  local timeStep = webots.wb_robot_get_basic_time_step()
  local camera_timeStep = math.max(33,timeStep)
  local lidar_timeStep = math.max(25,timeStep)
  Body.timeStep = timeStep

	-- Note: weirdness maybe...
  get_time = webots.wb_robot_get_time
	Body.get_sim_time = get_time
	Body.get_time = wcm.get_robot_t

  local depth_torch, depth_byte, depth_adj

  -- Set the correct servo properties for Webots
  servo.direction = vector.new({
    1,-1,-1,-1,1
  })
  assert(#servo.direction==nJoint,'Bad servo direction!')
  servo.offset = vector.zeros(nJoint)
  assert(#servo.offset==nJoint,'Bad servo offsets!')

  -- Setup the webots tags
  local tags = {}

  -- Ability to turn on/off items
  local t_last_keypress = get_time()
  -- Enable the keyboard 100ms
  webots.wb_robot_keyboard_enable( 100 )
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

  Body.entry = function()
    -- Grab the joints
  	tags.joints = {}
  	for i=1,nJoint do
      local name = 'arm'..i
  		tags.joints[i] = webots.wb_robot_get_device(name)
  		if tags.joints[i]>0 then
        webots.wb_motor_set_velocity(tags.joints[i], 0.5)
        webots.wb_motor_enable_position(tags.joints[i],Body.timeStep)
  		else
  			print('Joint '..i..' not found')
  		end
  	end

    -- Grab the wheels
  	tags.wheels = {}
  	for i=1,4 do
      local name = 'wheel'..i
  		tags.wheels[i] = webots.wb_robot_get_device(name)
  		if tags.joints[i]<0 then
  			print('Wheel '..i..' not found')
  		end
  	end

    -- Grab the fingers
    tags.fingers = {}
  	for i=1,2 do
      local name = 'finger'..i
  		tags.fingers[i] = webots.wb_robot_get_device(name)
  		if tags.joints[i]<0 then
        print('Finger '..i..' not found')
      else
        webots.wb_motor_set_velocity(tags.fingers[i],0.03)
      end
  	end

    -- Acquire sensor tags
		tags.gps = webots.wb_robot_get_device("GPS")
		tags.compass = webots.wb_robot_get_device("compass")
		tags.hand_camera = webots.wb_robot_get_device("HandCamera")
		tags.lidar = webots.wb_robot_get_device("lidar")
		tags.kinect = webots.wb_robot_get_device("kinect")

		-- Grab the original pose in the world
		key_action.p(true)
		-- Step the simulation
		webots.wb_robot_step(Body.timeStep)
		-- Form and set the pose
		local gps     = webots.wb_gps_get_values(tags.gps)
		local compass = webots.wb_compass_get_values(tags.compass)
		local angle   = math.atan2( compass[3], compass[1] )
		local pose    = vector.pose{gps[3], gps[1], util.mod_angle(angle + 90*DEG_TO_RAD)}
		wcm.set_robot_initialpose( pose )

		-- Enable sensors
		key_action.p(ENABLE_POSE)
		key_action.c(ENABLE_CAMERA)
		key_action.l(ENABLE_LIDAR)
		key_action.k(ENABLE_KINECT)

		-- Step it again
    webots.wb_robot_step(Body.timeStep)

    -- Read values
		local vals = {}
    for idx, jtag in pairs(tags.joints) do
      local val = webots.wb_motor_get_position( jtag )
      -- Take care of nan
      if val~=val then val = 0 end
      val = servo.direction[idx] * (val - servo.offset[idx])
			vals[idx] = val
    end
		jcm.set_sensor_position(vals)
		jcm.set_actuator_command_position(vals)
		-- Reset the body velocity
		mcm.set_walk_vel(vector.zeros(3))
  end

  local neg_inf, pos_inf = -1/0, 1/0
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


	Body.nop = function()
    -- Step only
    if webots.wb_robot_step(Body.timeStep) < 0 then
			print('EPIC FAIL NOP')
			os.exit()
		end
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
		local im_top = Body.get_img_top()
		lV.entry(Config.vision[1])
		local dbg_top = lV.update(im_top)
		-- Send all the debug information
		s_t:send(top_debug)
	end

  Body.update = function()

    -- Write arm commands
    local cmds = jcm.get_actuator_command_position()
    for i,v in ipairs(cmds) do
      local jtag = tags.joints[i]
      if jtag then
        -- Push to webots
				local qi = v * servo.direction[i] + servo.offset[i]
        webots.wb_motor_set_position( jtag, qi )
      end
    end

    -- Base
    local vel = mcm.get_walk_vel()
    wheel_helper( unpack(vel) )

    -- Gripper
    local spacing = jcm.get_gripper_command_position()
    local width = math.max(math.min(spacing[1],0.025),0)
    webots.wb_motor_set_position(tags.fingers[1], width)
    webots.wb_motor_set_position(tags.fingers[2], width)

		-- Step the simulation, and shutdown if the update fails
		if webots.wb_robot_step(Body.timeStep) < 0 then
			print('EPIC FAIL UPDATE')
			os.exit()
		end
    local t = get_time()
		-- Shared time...
		--wcm.set_robot_t(t)

    -- Read values
		local positions = {}
    for idx, jtag in pairs(tags.joints) do
      local val = webots.wb_motor_get_position( jtag )
      local rad = servo.direction[idx] * val - servo.offset[idx]
			positions[idx] = rad
    end
		jcm.set_sensor_position(positions)

    -- Get sensors
		if ENABLE_POSE then
    	local gps     = webots.wb_gps_get_values(tags.gps)
    	local compass = webots.wb_compass_get_values(tags.compass)
    	local angle   = math.atan2( compass[3], compass[1] )
    	local pose    = vector.pose{gps[3], gps[1], util.mod_angle(angle + 90*DEG_TO_RAD)}
			wcm.set_robot_gps( pose )
			-- If SLAM is not running, then use the Webots pose
			if wcm.get_map_enable_slam()==0 then wcm.set_robot_pose( pose ) end
		end
    -- Grab a camera frame
    if ENABLE_CAMERA then
      local camera_fr = webots.to_rgb(tags.hand_camera)
      local w = webots.wb_camera_get_width(tags.hand_camera)
      local h = webots.wb_camera_get_height(tags.hand_camera)
			local camera_arr = carray.byte( camera_fr, w*h*3 )
			local meta = {
				t = get_time(),
				n = #camera_arr,
				w = w,
				h = h,
				c = 'jpeg'
			}
			-- Send frame locally?
			--camera0_ch:send{mp.pack(meta),tostring(camera_arr)}
			-- Broadcast the frame
      local jpeg_fr = c_rgb:compress(camera_fr,w,h)
			camera0_ch:send{mp.pack(meta),jpeg_fr}
    end
    -- Grab a lidar scan
    if ENABLE_LIDAR then
      local w = webots.wb_camera_get_width(tags.lidar)
      local lidar_fr = webots.wb_camera_get_range_image(tags.lidar)
      local lidar_array = carray.float( lidar_fr, w )
			-- Send the message on the lidar channel
			local meta = {}
			meta.t     = get_time()
			meta.n     = w
			meta.res   = 360 / 1440
			meta.pose  = wcm.get_robot_pose()
			meta.angle = 0 -- no actuation
			lidar0_ch:send{mp.pack(meta),tostring(lidar_array)}
    end
    -- Grab kinect RGBD data
    if ENABLE_KINECT then
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

  Body.exit = function()
  end

end

-- Exports
Body.servo = servo
-- Working on time...
Body.get_time = get_time
Body.nJoint = nJoint

return Body
