#!/usr/local/bin/luajit
dofile'../../include.lua'
local util       = require'util'
local simple_ipc = require'simple_ipc'
local lM  = require'libMicrostrain'
local lM2  = require'libMicrostrain2'
local vector = require'vector'

local TEST_M2 = true
local libMicrostrain
if TEST_M2 then libMicrostrain = lM2 end


local imu = assert(libMicrostrain.new_microstrain(
--  '/dev/cu.usbmodem1421', 921600 )
  '/dev/ttyACM0'), 'No imu present!')

util.ptable(imu)

-- Print info
print('Opened Microstrain')
--imu:get_info()
--print(table.concat(imu.information,'\n'))

-- Set up the defaults:
--libMicrostrain.configure(imu,true)
--os.exit()

-- Change the baud rate to fastest for this session
--libMicrostrain.change_baud(imu)
--os.exit()

-- Turn on the stream

local t_debug = -math.huge
local t0 = unix.time()

print('ahrs_on')
imu:ahrs_on()
local cnt = 0
local running = true
while running do
  cnt = cnt + 1
	--print('read_ahrs')
	local acc, gyr, del_gyr, rpy, mag = imu:read_ahrs()
	local t = unix.time()
	if t - t_debug > 1 then
		print()
		print('rpy', rpy[0]*180/math.pi, rpy[1]*180/math.pi, rpy[2]*180/math.pi)
		print('gyr', gyr[0]*180/math.pi, gyr[1]*180/math.pi, gyr[2]*180/math.pi)
		print('acc', acc[0], acc[1], acc[2])
		t_debug = t
	end
  if t-t0>30 then running = false end
end
print('ahrs_off!')
imu:ahrs_off()

imu:close()
os.exit()
