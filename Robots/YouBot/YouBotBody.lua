-- Configuration
local ENABLE_CAMERA = true
local ENABLE_LIDAR  = true

local Body = {}

-- Useful constants
local DEG_TO_RAD = math.pi/180
local RAD_TO_DEG = 180/math.pi
Body.DEG_TO_RAD = DEG_TO_RAD
Body.RAD_TO_DEG = RAD_TO_DEG

-- Utilities
local unix         = require'unix'
local vector       = require'vector'
local quaternion   = require'quaternion'
local util         = require'util'

-- Kinamatics
local Transform = require'Transform'
local K = require'YouBotKinematics'
Body.Kinematics = K

-- Get time (for the real robot)
local get_time = unix.time

-- Shared memory
require'jcm'
require'mcm'

-- Five degree of freedom arm
local nJoint = 5

-- Table of servo properties
local servo = {}
servo.min_rad = vector.new({
  0,0,0,0,0
})*DEG_TO_RAD
assert(#servo.min_rad==nJoint,'Bad servo min_rad!')

servo.max_rad = vector.new({
  145,145,120,120,100
})*DEG_TO_RAD
assert(#servo.max_rad==nJoint,'Bad servo max_rad!')

-- NOTE: Servo direction is webots/real robot specific
servo.direction = vector.new({
  1,1,-1,1,1
})
assert(#servo.direction==nJoint,'Bad servo direction!')

-- Convienence functions for each joint
local jointNames = {
	"ShoulderYaw", "ShoulderPitch", "Elbow", "WristPitch","WristYaw"
}
assert(nJoint==#jointNames,'bad jointNames!')
for i,v in ipairs(jointNames) do
  local idx = i
  Body['set_'..v:lower()] = function(val)
    jcm.actuatorPtr.command_position[idx] = val
  end
  Body['get_'..v:lower()] = function()
    return jcm.sensorPtr.position[idx]
  end
end

-- Convience function for each joint sensor
for i,v in ipairs{'torque','velocity','position'} do
  Body['get_'..v] = jcm['get_sensor_'..v]
  Body['set_'..v] = jcm['set_sensor_'..v]
end
Body.set_command_position = jcm.set_actuator_command_position
Body.get_command_position = jcm.get_actuator_command_position

-- Base convience
Body.set_velocity = function(x,y,a)
  mcm.set_walk_vel{x,y,a}
end
Body.get_velocity = mcm.get_walk_vel

-- Entry initializes the hardware of the robot
Body.entry = function()
  
  kuka = require'kuka'
  
  kuka.init_base()
  kuka.init_arm()
  -- kuka.calibrate_arm()
  
  -- Set the initial joint command angles, so we have no jerk initially
  local init_pos = {}
  for i=1,nJoint do
    local rad = kuka.get_arm_position(i) * servo.direction[i]
    init_pos[i] = rad
  end
  jcm.set_actuator_command_position(init_pos)
  
end

-- Update speaks to the hardware of the robot
Body.update_cycle = 1 -- milliseconds
Body.update = function()
  -- Get joint readings
  local rad,mps,nm = {},{},{}
  for i=1,nJoint do
    rad[i] = kuka.get_arm_position(i) * servo.direction[i]
    mps[i] = kuka.get_arm_velocity(i)
    nm[i]  = kuka.get_arm_torque(i)
  end
  -- Set shm
  jcm.set_sensor_position(rad)
  jcm.set_sensor_velocity(mps)
  jcm.set_sensor_torque(nm)
  
  -- Set joints from shared memory
  local desired_pos = jcm.get_actuator_command_position()
  for i=1,nJoint do
    local v = desired_pos[i]
    -- Clamp the angle
    local val = math.max(math.min(v,servo.max_rad[i]),servo.min_rad[i])
    -- Put into the right direction
    val = val * servo.direction[i]
    -- Set the kuka arm
    kuka.set_arm_angle(i,val)
  end
  
  -- Set base from shared memory
  local vel = mcm.get_walk_vel()
  kuka.set_base_velocity( unpack(vel) )
  
end

-- Exit gracefully shuts down the hardware of the robot
Body.exit = function()
  kuka.shutdown_arm()
  kuka.shutdown_base()
end


-- Webots overrides
if IS_WEBOTS then
  
  webots = require'webots'
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
  
  -- Ability to turn on/off items
  local t_last_keypress = get_time()
  local key_action = {
    l = function()
      if ENABLE_LIDAR then
        print(util.color('LIDAR disabled!','yellow'))
        webots.wb_camera_disable(tags.lidar)
        ENABLE_LIDAR = false
      else
        print(util.color('LIDAR enabled!','green'))
        webots.wb_camera_enable(tags.lidar,lidar_timeStep)
        ENABLE_LIDAR = true
      end
    end,
    c = function()
      if ENABLE_CAMERA then
        print(util.color('CAMERA disabled!','yellow'))
        webots.wb_camera_disable(tags.hand_camera)
        ENABLE_CAMERA = false
      else
        print(util.color('CAMERA enabled!','green'))
        webots.wb_camera_enable(tags.hand_camera,camera_timeStep)
        ENABLE_CAMERA = true
      end
    end
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
    
    -- Body Sensors
	  tags.gps = webots.wb_robot_get_device("GPS")
	  webots.wb_gps_enable(tags.gps, timeStep)
	  tags.compass = webots.wb_robot_get_device("Compass")
	  webots.wb_compass_enable(tags.compass, timeStep)

    -- Grab the camera
    tags.hand_camera = webots.wb_robot_get_device("HandCamera")
    if ENABLE_CAMERA then
      webots.wb_camera_enable(tags.hand_camera, camera_timeStep)
    end

    -- Grab the hokuyo
    tags.lidar = webots.wb_robot_get_device("lidar")
    if ENABLE_LIDAR then
      webots.wb_camera_enable(tags.lidar, lidar_timeStep)
    end
    
    -- Step the simulation
		webots.wb_robot_step(Body.timeStep)
    webots.wb_robot_step(Body.timeStep)
    
    -- Read values
    for idx, jtag in pairs(tags.joints) do
      local val = webots.wb_motor_get_position( jtag )
      -- Take care of nan
      if val~=val then
        val = 0
      else
        val = servo.direction[idx] * val
      end
      jcm.sensorPtr.position[idx] = val
      jcm.actuatorPtr.command_position[idx] = val
    end
    
    -- Enable the keyboard 100ms
    webots.wb_robot_keyboard_enable( 100 )

  end
  
  local neg_inf, pos_inf = -1/0, 1/0
  local wb_wheel_speed = function(tag,speed)
    if speed > 0 then
      webots.wb_motor_set_position(tag,pos_inf)
    else
      webots.wb_motor_set_position(tag,neg_inf)
    end
    webots.wb_motor_set_velocity(tag,math.abs(speed))
  end
  local wheel_helper = function(vx,vy,va)
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
  
  Body.update = function()
    -- Write values
    local cmds = Body.get_command_position()
    for i,v in ipairs(cmds) do
      local jtag = tags.joints[i]
      if jtag then
        local pos = servo.direction[i] * v
        webots.wb_motor_set_position( jtag, pos )
      end
    end
    
    -- Base
    local vel = mcm.get_walk_vel()
    wheel_helper( unpack(vel) )
    
		-- Step the simulation, and shutdown if the update fails
		if webots.wb_robot_step(Body.timeStep) < 0 then os.exit() end
    local t = get_time()
    
    -- Read values
    for idx, jtag in pairs(tags.joints) do
      local val = webots.wb_motor_get_position( jtag )
      local rad = servo.direction[idx] * val
      jcm.sensorPtr.position[idx] = rad
    end
    
    -- Get sensors
    local gps     = webots.wb_gps_get_values(tags.gps)
    local compass = webots.wb_compass_get_values(tags.compass)

    -- Grab a camera frame
    if ENABLE_CAMERA then
      local camera_fr = webots.to_rgb(tags.hand_camera)
    end
    -- Grab a lidar scan
    if ENABLE_LIDAR then
      local lidar_fr = webots.wb_camera_get_range_image(tags.lidar)
    end

    -- Grab keyboard input, for modifying items
    local key_code = webots.wb_robot_keyboard_get_key()
    local key_char = string.char(key_code)
    local key_char_lower = string.lower(key_char)
    if t-t_last_keypress>1 and key_code and key_action[key_char_lower] then
      key_action[key_char_lower]()
      t_last_keypress = t
    end
    
  end
  
  Body.exit = function()
  end
  
end

Body.get_time = get_time

return Body
