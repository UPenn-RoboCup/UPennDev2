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
local Transform    = require'Transform'
local util         = require'util'

-- Get time (for the real robot)
local get_time = unix.time

-- KUKA interface
local kuka = require'kuka'

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
  145,145,100,100,100
})*DEG_TO_RAD
assert(#servo.max_rad==nJoint,'Bad servo max_rad!')

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

-- Entry initializes the hardware of the robot
Body.entry = function()
  kuka.init_base()
  kuka.init_arm()
  -- kuka.calibrate_arm()
  
  -- Set the initial joint command angles, so we have no jerk initially
  local init_pos = {}
  for i=1,nJoint do
    local rad = kuka.get_arm_position(i)
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
    rad[i] = kuka.get_arm_position(i)
    mps[i] = kuka.get_arm_velocity(i)
    nm[i] = kuka.get_arm_torque(i)
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

return Body
