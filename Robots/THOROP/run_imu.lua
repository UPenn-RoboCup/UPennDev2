-- DCM is a thread or standalone
local CTX, metadata = ...
-- Still need our library paths set
dofile'include.lua'
--assert(ffi, 'IMU | Please use LuaJIT :). Lua support in the near future')
-- Going to be threading this
local si = require'simple_ipc'
-- Import the context
print('CTX',CTX,type(CTX))
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
-- Modules
require'dcm'
local lM = require'libMicrostrain'
-- TODO: Remove carray when using LuaJIT
local carray = require'carray'
local ptable = require'util'.ptable
local usleep, get_time = unix.usleep, unix.time
-- Open the device
local microstrain = libMicrostrain.new_microstrain'/dev/ttyACM0'
-- Turn it on
-- TODO: Read and check settings...
microstrain:ahrs_on()
-- Cache the typical commands quickly
local rpy_ptr   = dcm.sensorPtr.rpy
local gyro_ptr  = dcm.sensorPtr.gyro
local gyro, rpy = {}, {}
local function do_read ()
	unix.select({microstrain.fd})
	local buf = unix.read(microstrain.fd)
	-- TODO: Check the size of the buffer, too
	if buf then
		local _gyro = carray.float(buf:sub( 7,18):reverse())
		local _rpy  = carray.float(buf:sub(21,32):reverse())
		-- Save locally
		rpy = {_rpy[2], _rpy[3], -_rpy[1]}
		gyro = {_gyro[2],_gyro[3],-_gyro[1]}
		-- set to memory
		dcm.set_sensor_rpy(rpy)
		dcm.set_sensor_gyro(gyro)
	end
end
-- Collect garbage before starting
collectgarbage()
-- Begin infinite loop
local t0 = get_time()
local t_debug = t0
while true do
	local t = get_time()
	local t_diff = t-t0
	t0 = t
	--------------------
	-- Periodic Debug --
	--------------------
  if t - t_debug>1 then
	  print('\IMU | t_diff', t_diff, 1 / t_diff)
    ptable(positions)
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
end

microstrain:ahrs_off()
microstrain:close()
