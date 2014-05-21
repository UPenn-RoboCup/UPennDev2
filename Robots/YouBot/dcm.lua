--------------------------------
-- Joint Communication Module
-- (c) 2013 Stephen McGill
--------------------------------
local Config = Config or require'Config'
assert(Config, 'DCM requires a config, since it defines joints!')
local nJoint = Config.nJoint
local memory = require'memory'
local vector = require'vector'

local shared_data = {}
local shared_data_sz = {}

-- Sensors from the robot
shared_data.sensor = {}
for _, sensor in ipairs(Config.sensors) do
  shared_data.sensor[sensor] = vector.zeros(Config.nJoint)
end
shared_data.sensor.gripper = vector.zeros(1)

-- Raw inertial readings
shared_data.sensor.accelerometer = vector.zeros(3)
shared_data.sensor.gyro          = vector.zeros(3)
shared_data.sensor.compass       = vector.zeros(3)
-- Filtered Roll/Pitch/Yaw
shared_data.sensor.rpy           = vector.zeros(3)
-- Battery level (in volts)
shared_data.sensor.battery       = vector.zeros(1)

--  Write to the motors
shared_data.actuator = {}
for _, actuator in ipairs(Config.actuators) do
  shared_data.actuator[actuator] = vector.zeros(Config.nJoint)
end
shared_data.actuator.command_gripper = vector.zeros(1)

------------------------
-- Call the initializer
memory.init_shm_segment(..., shared_data, shared_data_sz)
