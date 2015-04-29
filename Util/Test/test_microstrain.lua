dofile'../../include.lua'
local Body       = require'Body'
local signal     = require'signal'
local carray     = require'carray'
local util       = require'util'
local simple_ipc = require'simple_ipc'
local libMicrostrain  = require'libMicrostrain'
local vector = require'vector'

local RAD_TO_DEG = Body.RAD_TO_DEG

local imu = libMicrostrain.new_microstrain(
--  '/dev/cu.usbmodem1421', 921600 )
  '/dev/ttyACM0')

if not imu then
  print('No imu present!')
  os.exit()
end

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
