--------------------------------
-- Joint Communication Module
-- (c) 2013 Stephen McGill
--------------------------------

-- SJ: hacked for DARWIN-OP!


local nJoint = 20
local memory = require'memory'
local vector = require'vector'
--local lD = require'libDynamixel'

local char=string.char

lD={}
lD.registers_sensor = {
        'position', 'velocity', 'current', 'load', 'voltage', 'temperature',
}

local nx_registers = {

	-- New API --
	-- ENTER EEPROM AREA

	-- General Operation information
	['model_number']  = {char(0x00,0x00),2},
	['model_information'] = {char(0x02,0x00),4},
	['firmware'] =   {char(0x06,0x00),1},
	['id'] =   {char(0x07,0x00),1},
	-- Baud
	--[[
	0: 2400 ,1: 57600, 2: 115200, 3: 1Mbps, 4: 2Mbps
	5: 3Mbps, 6: 4Mbps, 7: 4.5Mbps, 8: 10.5Mbps
	--]]
	['baud_rate'] = {char(0x08,0x00),1},
	-- Delay in us: wish to have zero
	['return_delay_time'] = {char(9,0),1},

	-- Operation Mode
	-- Mode 0: Torque Control
	-- Mode 1: Velocity Control
	-- Mode 2: Position Control
	-- Mode 3: position-Velocity Control
	['mode'] = {char(0x0B,0x00),1},
	['homing_offset'] = {char(13,0x00),4},

	-- Limits
	['max_temperature'] = {char(0x15,0x00,1)},
	['max_voltage'] = {char(0x16,0x00),2},
	['min_voltage'] = {char(0x18,0x00),2},
	['max_acceleration'] = {char(0x1A,0x00),4},
	['max_torque'] = {char(0x1E,0x00),2},
	['max_velocity'] = {char(0x20,0x00),4},
	['max_position'] = {char(0x24,0x00),4},
	['min_position'] = {char(0x28,0x00),4},
	--
	['data1_mode'] = {char(44,0x00),1},
	['data2_mode'] = {char(45,0x00),1},
	['data3_mode'] = {char(46,0x00),1},
	['data4_mode'] = {char(47,0x00),1},
	--
	['shutdown'] = {char(48,0x00),1},
	['alarm_shutdown'] = {char(48,0x00),1},

	-- ENTER RAM AREA
	['torque_enable'] = {char(0x32,0x02),1},
	-- LED lighting
	['led_red'] = {char(0x33,0x02),1},
	['led_green'] = {char(0x34,0x02),1},
	['led_blue'] = {char(0x35,0x02),1},
	-- Velocity PID Gains (position control mode)
	['velocity_i'] = {char(0x4A,0x02),2},
	['velocity_p'] = {char(0x46,0x02),2},
	--['velocity_d'] = {char(0x4C,0x02),2},
	-- Position PID Gains (position control mode)
	['position_p'] = {char(0x52,0x02),2},
	['position_i'] = {char(0x50,0x02),2},
	['position_d'] = {char(0x4E,0x02),2},

	-- Position Commands (position control mode)
	['command_position'] = {char(0x54,0x02),4},
	['command_velocity'] = {char(0x58,0x02),4},
	['command_acceleration'] = {char(0x5E,0x02),4},

	-- Low Pass Fitler settings
	['position_lpf'] = {char(0x42,0x02),4},
	['velocity_lpf'] = {char(0x46,0x02),4},
	-- Feed Forward mechanism
	['acceleration_ff'] = {char(0x3A,0x02),4},
	['velocity_ff'] = {char(0x3E,0x02),4},

	-- Torque options --
	-- Commanded Torque (torque control mode)
	['command_torque'] = {char(0x5C,0x02),4},
	-- Current (V=iR) PI Gains (torque control mode)
	['current_p'] = {char(0x38,0x02),2},
	['current_i'] = {char(0x36,0x02),2},


	-- Present information
	['position'] = {char(0x63,0x02),4},
	['velocity'] = {char(0x67,0x02),4},
	['current'] = {char(0x6D,0x02),2},
	['load'] = {char(0x6B,0x02),2},
	['voltage'] = {char(0x6F,0x02),2},
	['temperature'] = {char(0x71,0x02),1},

	-- External Data
	['data1'] = {char(0x72,0x02),2},
	['data2'] = {char(0x74,0x02),2},
	['data3'] = {char(0x76,0x02),2},
	['data4'] = {char(0x78,0x02),2},
	-- All data
	['data']  = {char(0x72,0x02),8},
	['indirect_data']  = {char(0x7A,0x02),1},

	-- Status return
	['status_return_level'] = {char(0x7B,0x03),1},
	['hardware_error'] = {char(0x7C,0x03),2},
}
lD.nx_registers = nx_registers





local shared_data = {}
local shared_data_sz = {}

-- Sensors from the robot
shared_data.sensor = {}
-- Setup from libDynamixel the read only sensor values
for _, sensor in ipairs(lD.registers_sensor) do
  shared_data.sensor[sensor] = vector.zeros(nJoint)
end

-- Force Torque
shared_data.sensor.lfoot = vector.zeros(6)
shared_data.sensor.rfoot = vector.zeros(6)
shared_data.sensor.lzmp = vector.zeros(2)
shared_data.sensor.rzmp = vector.zeros(2)

-- These should not be tied in with the motor readings,
-- so they come after the read/tread setup
-- Raw inertial readings
shared_data.sensor.accelerometer = vector.zeros(3)
shared_data.sensor.gyro          = vector.zeros(3)
shared_data.sensor.magnetometer  = vector.zeros(3)
shared_data.sensor.imu_t  = vector.zeros(1) --we timestamp IMU so that we can run IMU later than state wizard
shared_data.sensor.imu_t0  = vector.zeros(1) --run_imu start time
-- Filtered Roll/Pitch/Yaw
shared_data.sensor.rpy           = vector.zeros(3)
-- Battery level (in volts)
shared_data.sensor.battery       = vector.zeros(1)
shared_data.sensor.compass       = vector.zeros(3)

-- Sensors from the robot
shared_data.tsensor = {}
-- Setup from libDynamixel the read only sensor values
for name, vec in pairs(shared_data.sensor) do
  shared_data.tsensor[name] = vector.zeros(#vec)
end

--  Write to the motors
shared_data.actuator = {}


-- Additional memory variables (to make old op code work)
shared_data.actuator.read_enable = vector.zeros(1) 
shared_data.actuator.torqueEnable = vector.zeros(1) 
shared_data.actuator.gainLevel = vector.zeros(1) 

shared_data.actuator.torqueEnableChanged = vector.zeros(1) 
shared_data.actuator.hardnessChanged = vector.zeros(1) 
shared_data.actuator.gainChanged = vector.zeros(1) 

shared_data.actuator.offset = vector.zeros(nJoint)
shared_data.actuator.bias = vector.zeros(nJoint)
shared_data.actuator.hardness = vector.ones(nJoint)

shared_data.sensor.updatedCount  = vector.zeros(1) 




-- Setup from libDynamixel every write item
-- TODO: Separate parameters? RAM/ROM
for reg, v in pairs(lD.nx_registers) do
  -- Check that it is not a sensor
  local is_sensor
  for _, s in ipairs(lD.registers_sensor) do
    if s==reg then is_sensor = true; break; end
  end
  if not is_sensor then
    shared_data.actuator[reg] = vector.zeros(nJoint)
  end
end

------------------------
-- Call the initializer
memory.init_shm_segment(..., shared_data, shared_data_sz)
