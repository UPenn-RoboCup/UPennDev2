-- DCM is a thread or standalone
local CTX, metadata = ...
-- Still need our library paths set
dofile'include.lua'
--assert(ffi, 'IMU | Please use LuaJIT :). Lua support in the near future')
-- Going to be threading this
local si = require'simple_ipc'
-- Import the context
local parent_ch, IS_THREAD
if CTX and not arg then
	IS_THREAD = true
	si.import_context(CTX)
	-- Communicate with the master thread
	parent_ch = si.new_pair(metadata.ch_name)
else
	metadata = Config.imu
	parent_ch = si.new_subscriber'imu!'
end
-- Fallback on undefined metadata
assert(metadata, 'IMU | No metadata found!')
local running = true
local function shutdown()
	running = false
end
if not IS_THREAD then
  signal = require'signal'
  signal.signal("SIGINT", shutdown)
  signal.signal("SIGTERM", shutdown)
end

-- Modules
require'dcm'
local lM = require'libMicrostrain'
local vector = require'vector'
local get_time = unix.time
-- Open the device
local microstrain = lM.new_microstrain('/dev/ttyACM0', OPERATING_SYSTEM~='darwin' and 921600)
-- Turn it on
-- TODO: Read and check settings...
microstrain:ahrs_on()
-- Cache the typical commands quickly
local acc_ptr  = dcm.sensorPtr.accelerometer
local gyro_ptr = dcm.sensorPtr.gyro
local mag_ptr  = dcm.sensorPtr.magnetometer
local rpy_ptr  = dcm.sensorPtr.rpy
local acc, gyro, mag, rpy = vector.new(3), vector.new(3), vector.new(3), vector.new(3)
local function do_read ()
	-- Get the accelerometer, gyro, magnetometer, and euler angles
	local a, g, m, e = microstrain:read_ahrs()
	if not a then return end
	-- Quickly set in shared memory
	acc_ptr[0], acc_ptr[1], acc_ptr[2] = a[1], a[2], -a[0]
	gyro_ptr[0], gyro_ptr[1], gyro_ptr[2] = g[1], g[2], -g[0]
	mag_ptr[0], mag_ptr[1], mag_ptr[2] = m[1], m[2], -m[0]
	rpy_ptr[0], rpy_ptr[1], rpy_ptr[2] = e[1], e[2], -e[0]
	-- Save locally
	acc[1], acc[2], acc[3] = a[1], a[2], -a[0]
	gyro[1], gyro[2], gyro[3] = g[1], g[2], -g[0]
	mag[1], mag[2], mag[3] = m[1], m[2], -m[0]
	rpy[1], rpy[2], rpy[3] = e[1], e[2], -e[0]
	--[[
	-- Save locally
	acc  = {a[1], a[2], -a[0]}
	gyro = {g[1], g[2], -g[0]}
	mag  = {m[1], m[2], -m[0]}
	rpy  = {e[1], e[2], -e[0]}
	-- Set to memory
	dcm.set_sensor_accelerometer(acc)
	dcm.set_sensor_gyro(gyro)
	dcm.set_sensor_magnetometer(mag)
	dcm.set_sensor_rpy(rpy)
	--]]
end
-- Collect garbage before starting
collectgarbage()
-- Begin infinite loop
local t0, t = get_time()
local t_debug, t_last = t0, t0
while running do
	t_last = t
	t = get_time()
	--------------------
	-- Periodic Debug --
	--------------------
  if t - t_debug>1 then
		local kb = collectgarbage('count')
		local debug_str = {
			string.format('\nIMU | Uptime %.2f sec, Mem: %d kB', t-t0, kb),
			string.format('Acc (g): %.2f %.2f %.2f', unpack(acc)),
			string.format('Gyro (rad/s): %.2f %.2f %.2f', unpack(gyro)),
			string.format('Mag (Gauss): %.2f %.2f %.2f', unpack(mag)),
			string.format('RPY:  %.2f %.2f %.2f', unpack(rpy)),
		}
		print(table.concat(debug_str,'\n'))
    t_debug = t
  end
	-----------------
	-- Read Values --
	-----------------
	do_read()
	---------------------
	-- Parent Commands --
	---------------------
	local parent_msgs = parent_ch:receive(true)
	if parent_msgs then
		for _, msg in ipairs(parent_msgs) do
			if msg=='exit' then
				shutdown()
			end
		end
	end
	collectgarbage('step')
end

microstrain:ahrs_off()
microstrain:close()
if IS_THREAD then parent_ch:send'done' end
print('IMU | Exit')
