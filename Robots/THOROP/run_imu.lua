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
if not IS_THREAD then
  signal = require'signal'
  function shutdown ()
    running = false
  end
  signal.signal("SIGINT", shutdown)
  signal.signal("SIGTERM", shutdown)
end

-- Modules
require'dcm'
local lM = require'libMicrostrain'
-- TODO: Remove carray when using LuaJIT
local carray = require'carray'
local ptable = require'util'.ptable
local usleep, get_time = unix.usleep, unix.time
-- Open the device
local microstrain = lM.new_microstrain('/dev/ttyACM0', OPERATING_SYSTEM~='darwin' and 921600)
-- Turn it on
-- TODO: Read and check settings...
microstrain:ahrs_on()
-- Cache the typical commands quickly
local rpy_ptr   = dcm.sensorPtr.rpy
local gyro_ptr  = dcm.sensorPtr.gyro
local acc, gyro, mag, rpy
--local do_debug = false
local function do_read ()
	-- Get the accelerometer, gyro, magnetometer, and euler angles
	local a, g, m, e = microstrain:read_ahrs()
	-- Save locally
	acc = {a[2], a[3], -a[1]}
	gyro  = {g[2], g[3], -g[1]}
	mag   = {m[2], m[3], -m[1]}
	rpy   = {e[1], e[2], -e[0]}
	-- Set to memory
	dcm.set_sensor_accelerometer(acc)
	dcm.set_sensor_gyro(gyro)
	dcm.set_sensor_magnetometer(mag)
	dcm.set_sensor_rpy(rpy)
	--[[
	local buf = microstrain:read_ahrs()
	if do_debug then
		do_debug = false
		local hex = {}
		local bytes = {buf:byte(1, -1)}
		for i,v in ipairs(bytes) do
			table.insert(hex, string.format('0x%02X', v))
		end
		print(table.concat(hex,' '))
	end
	--]]
end
-- Collect garbage before starting
collectgarbage()
-- Begin infinite loop
local t0 = get_time()
local t_debug, t, t_diff = t0
while running do
	t = get_time()
	t_diff = t-t0
	t0 = t
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
		--do_debug = true
    t_debug = t
  end
	-----------------
	-- Read Values --
	-----------------
	do_read()
	---------------------
	-- Parent Commands --
	---------------------
	local parent_msg = parent_ch:receive(true)
	if parent_msg then
		if parent_msg=='exit' then
			bus:close()
			if IS_THREAD then parent_msg:send'done' end
			return
		else
			process_parent(parent_msg)
		end
	end
	collectgarbage('step')
end

microstrain:ahrs_off()
microstrain:close()
