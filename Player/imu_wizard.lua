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
local vector = require'vector'

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
local accel = vector.zeros(3)
local function process_imu(gyro,rpy)
  jcm.set_sensor_rpy(rpy:table())
  jcm.set_sensor_gyro(gyro:table())
end

-- Main thread
local cnt = 0
local t0 = unix.time()
local t_debug = t0
local print_rate = 2 --every 2 sec, print
local function main()
  while true do
    local t = unix.time()
    cnt = cnt+1
    local t_diff = t-t_debug
    if t_diff>print_rate then
      print( string.format('FPS: %.1f',cnt/t_diff) )
      print(accel)
      print(RAD_TO_DEG*math.atan2(accel[2],-accel[1]))
      t_debug = t
      cnt = 0
    end
    coroutine.yield()
  end
end

-- Service the microstrain
imu.callback = process_imu
libMicrostrain.service(imu,main)
