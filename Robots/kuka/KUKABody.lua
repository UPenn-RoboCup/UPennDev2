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

-- Shared memory
require'jcm'

-- Five degree of freedom arm
local nJoint = 5

-- Entry initializes the hardware of the robot
Body.entry = function()
  kuka.init_base()
  kuka.init_arm()
  -- kuka.calibrate_arm()
end

-- Update speaks to the hardware of the robot
Body.update = function()
  -- Get joints and place in shared memory
  for i=1,nJoint do
    local rad = kuka.get_arm_position(i)
    jcm.set_sensor_position(rad,i)
    local mps = kuka.get_arm_velocity(i)
    jcm.set_sensor_velocity(mps,i)
    local nm = kuka.get_arm_torque(i)
    jcm.set_sensor_torque(nm,i)
  end
  
  -- Set joints from shared memory
  local desired_pos = jcm.get_actuator_command_position()
  for i,v in ipairs(desired_pos) do
    kuka.set_arm_angle(i,v)
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