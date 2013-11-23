--------------------------------
-- Joint Communication Module
-- (c) 2013 Stephen McGill
--------------------------------
local memory = require'memory'
local vector = require'vector'
local nJoints = 40
local shared_data = {}
local shared_data_sz = {}

-- NOTE: Sensor and actuator names should not overlap!
-- This is because Body makes certain get/set assumptions

------------------------
--  Read from the motors/other sensors
shared_data.sensor          = {}
-- Position of joints in radians from the zero position
shared_data.sensor.position = vector.zeros( nJoints )
-- Velocity of joints is in radians per second
shared_data.sensor.velocity = vector.zeros( nJoints )
-- Load of joints is measured in percentage
shared_data.sensor.load     = vector.zeros( nJoints )
-- Temperature of joints is measured in Celsius
shared_data.sensor.temperature  = vector.zeros( nJoints )

-- Foot sensors (strain gauges; should be just 6, not quite 8...)
-- 8 is because 2 motors, with 4 ext data each
shared_data.sensor.lfoot = vector.zeros( 8 )
shared_data.sensor.rfoot = vector.zeros( 8 )
--[[
shared_data.sensor.lfoot_torque   = vector.zeros( 4 )
shared_data.sensor.rfoot_torque   = vector.zeros( 4 )
shared_data.sensor.lfoot_force    = vector.zeros( 4 )
shared_data.sensor.rfoot_force    = vector.zeros( 4 )
--]]
------------------------
-- Request reads from some motors
-- 0: do not read
-- 1: read once
-- 2: read continuously
shared_data.read = {}
-- Timestamps of the last read
shared_data.tread = {}
-- Timestamps of the last request
shared_data.trequest = {}
for k,v in pairs(shared_data.sensor) do
  if k:find'foot' then
    shared_data.read[k]     = vector.zeros(1)
    shared_data.tread[k]    = vector.zeros(1)
    shared_data.trequest[k] = vector.zeros(1)
  else
    shared_data.read[k] = v
    shared_data.tread[k] = v
    shared_data.trequest[k] = v
  end
end

-- These should not be tied in with the motor readings,
-- so they come after the read/tread setup
-- Raw inertial readings
shared_data.sensor.accelerometer = vector.zeros( 3 )
shared_data.sensor.gyro          = vector.zeros( 3 )
shared_data.sensor.compass       = vector.zeros( 3 )
-- Filtered Roll/Pitch/Yaw
shared_data.sensor.rpy           = vector.zeros( 3 )
-- Battery level (in volts)
shared_data.sensor.battery       = vector.zeros( 1 )

------------------------
--  Write to the motors/other actuators
shared_data.actuator                  = {}
-- Position of joints is in radians from the zero position
shared_data.actuator.command_position = vector.zeros( nJoints )
-- Velocity of joints is in radians from the zero position
shared_data.actuator.command_velocity = 17000*vector.ones( nJoints )
-- Velocity of joints is in radians from the zero position
shared_data.actuator.command_acceleration = 64*vector.ones( nJoints )
-- Torque enable of joints is 0 or 1
shared_data.actuator.torque_enable    = vector.zeros( nJoints )
-- Hardness of joints is legacy, but offers a simple abstraction of pid gains
shared_data.actuator.position_p       = 64*vector.ones( nJoints )

------------------------
-- Request writes to some motors
-- 0: do not write
-- 1: write once
-- 2: write continuously
shared_data.write = {}
shared_data.twrite = {}
for k,v in pairs(shared_data.actuator) do
  shared_data.write[k] = v
  shared_data.twrite[k] = v
end

-- Gripper only data Left, Right
shared_data.gripper = {}
-- Torque of joints is in mA (just for the gripper)
shared_data.gripper.command_torque = vector.zeros( 4 )
-- 0: Position mode, 1: torque mode
shared_data.gripper.torque_mode = vector.zeros( 4 )
-- Temperature
shared_data.gripper.temperature = vector.zeros( 4 )

------------------------
-- Call the initializer
memory.init_shm_segment(..., shared_data, shared_data_sz)
