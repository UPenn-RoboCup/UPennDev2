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
imu:get_info()
print(table.concat(imu.information,'\n'))

-- Set up the defaults:
--libMicrostrain.configure(imu,true)
--os.exit()

-- Change the baud rate to fastest for this session
--libMicrostrain.change_baud(imu)
--os.exit()

-- Turn on the stream
print('ahrs_on')
imu:ahrs_on()
local cnt = 0
local running = true
while running do
  cnt = cnt + 1
	print('read_ahrs')
	imu:read_ahrs()
  if cnt>5 then running = false end
end
print('ahrs_off!')
imu:ahrs_off()

imu:close()
os.exit()
