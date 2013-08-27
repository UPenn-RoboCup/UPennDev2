--------------------------------
-- Joint Communication Module
-- (c) 2013 Stephen McGill
--------------------------------
local memory = require'memory'
local nJoints = 40
local shared_data = {}
local shared_data_sz = {}

-- NOTE: Sensor and actuator names should not overlap!
-- This is because Body makes certain get/set assumptions

------------------------
--  Read from the motors/other sensors
shared_data.sensor               = {}
-- Position of joints is in radians from the zero position
shared_data.sensor.position      = vector.zeros( nJoints )
-- Velocity of joints is in radians per second
shared_data.sensor.velocity      = vector.zeros( nJoints )
-- Load of joints is measured in percentage
shared_data.sensor.load          = vector.zeros( nJoints )
-- Raw inertial readings
shared_data.sensor.accelerometer = vector.zeros( 3 )
shared_data.sensor.gyro          = vector.zeros( 3 )
shared_data.sensor.compass       = vector.zeros( 3 )
shared_data.sensor.imuAngle      = vector.zeros( 3 )
------------------------
-- Request reads from some motors
shared_data.read = {}
for k,v in pairs(shared_data.sensor)
  shared_data.read[k] = v
end

------------------------
--  Write to the motors/other actuators
shared_data.actuator                  = {}
-- Position of joints is in radians from the zero position
shared_data.actuator.command_position = vector.zeros( nJoints )
-- Position of joints is in radians from the zero position
shared_data.actuator.command_velocity = vector.zeros( nJoints )
-- Torque enable of joints is 0 or 1
shared_data.actuator.torque_enable    = vector.zeros( nJoints )
-- Hardness of joints is legacy, but offers a simple abstraction of pid gains
shared_data.actuator.hardness         = vector.zeros( nJoints )

------------------------
-- Request writes to some motors
shared_data.write = {}
for k,v in pairs(shared_data.actuator)
  shared_data.write[k] = v
end

------------------------
-- Call the initializer
memory.init_shm_segment(..., shared_data, shared_data_sz)