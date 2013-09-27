-----------------------------------
-- Dynamixel Motor Communication --
-- Performs callbacks for        --
-- libDynamixel Servicing        --
-- (c) Stephen McGill, 2013      --
-----------------------------------
dofile'include.lua'

-- Libraries
local Body   = require'Body'
local signal = require'signal'
local vector = require'vector'
local util   = require'util'
local simple_ipc = require'simple_ipc'
local libMicrostrain = require'libMicrostrain'

local DEG_TO_RAD = Body.DEG_TO_RAD
local RAD_TO_DEG = Body.RAD_TO_DEG
require'jcm'

local device_tty = '/dev/ttyACM0'
if OPERATING_SYSTEM=='darwin' then
  device_tty = '/dev/cu.usbmodem1421'
end

local imu = libMicrostrain.new_microstrain(
  device_tty, 115200 )
assert(imu,'Could not open IMU.')

-- Print info
print('Opened Microstrain')
print(table.concat(imu.information,'\n'))

-- Signal handling
-- Ensure that we shutdown the device properly
function shutdown()
  print'Shutting down the Microstrain...'
  imu:ahrs_off()
  imu:close()
  os.exit()
end
signal.signal("SIGINT",  shutdown)
signal.signal("SIGTERM", shutdown)

-- Callback
local function process_imu(acc,gyr)
  print('Acc',acc[1],acc[2],acc[3])
  print('Gyr',gyr[1],gyr[2],gyr[3])
end

-- Service the microstrain
imu.callback = process_imu
libMicrostrain.service(imu)