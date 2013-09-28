dofile'../../include.lua'
local Body       = require'Body'
local signal     = require'signal'
local carray     = require'carray'
local mp         = require'msgpack'
local util       = require'util'
local simple_ipc = require'simple_ipc'
local libMicrostrain  = require'libMicrostrain'
local vector = require'vector'

local RAD_TO_DEG = Body.RAD_TO_DEG

local imu = libMicrostrain.new_microstrain(
  '/dev/cu.usbmodem1421', 115200 )

if not imu then
  print('No imu present!')
  os.exit()
end

--util.ptable(imu)

-- Print info
print('Opened Microstrain')
print(table.concat(imu.information,'\n'))

-- Set up the defaults:
--libMicrostrain.configure(imu)
--os.exit()

-- Change the baud rate to fastest for this session
--libMicrostrain.change_baud(imu)
--os.exit()

-- Turn on the stream
imu:ahrs_on()
local cnt = 0
while true do
  local ret_fd = unix.select( {imu.fd} )
  print('READING')
  res = unix.read(imu.fd)
  assert(res)
  local response = {string.byte(res,1,-1)}
  for i,b in ipairs(response) do print( string.format('%d: %02X %d',i,b,b) ) end
  local gyr_str = res:sub(7,18)
  local gyro = carray.float( gyr_str:reverse() )
  print( string.format('GYRO: %g %g %g',unpack(gyro:table())))

  local rpy_str = res:sub(21,32)
  local rpy = carray.float( rpy_str:reverse() )
  rpy = vector.new( rpy:table() )
  print( 'RPY:', rpy*RAD_TO_DEG )
--[[
  local acc_str = res:sub(7,18)
  for i,b in ipairs{acc_str:byte(1,-1)} do
    print( string.format('acc %d: %02X %d',i,b,b) )
  end
  local acc = carray.float( acc_str )
  for i=1,#acc do
    print('acc',acc[i])
  end
  local acc_rev = carray.float( acc_str:reverse() )
  for i=1,#acc_rev do
    print('acc_rev',acc_rev[i])
  end
--]]
  cnt = cnt+1
  if cnt>5 then
    imu:ahrs_off()
    break
  end
end
print('done!')

-- test 1 sec timeout
local ret_fd = unix.select( {imu.fd}, 1 )
print('timeout!',ret_fd)

imu:close()
os.exit()

-- Ensure that we shutdown the devices properly
function shutdown()
  print'Shutting down the Hokuyos...'
  for i,h in ipairs(hokuyos) do
    h:stream_off()
    h:close()
    print('Closed Hokuyo',i)
  end
  os.exit()
end
signal.signal("SIGINT", shutdown)
signal.signal("SIGTERM", shutdown)

-- Begin to service
os.execute('clear')
assert(#hokuyos>0,"No hokuyos detected!")
print( util.color('Servicing '..#hokuyos..' Hokuyos','green') )

local main = function()
  local main_cnt = 0
  local t0 = Body.get_time()
  while true do
    main_cnt = main_cnt + 1
    local t_now = Body.get_time()
    local t_diff = t_now - t0
    if t_diff>1 then
      local debug_str = string.format('\nMain loop: %7.2f Hz',main_cnt/t_diff)
      debug_str = util.color(debug_str,'yellow')
      for i,h in ipairs(hokuyos) do
        debug_str = debug_str..string.format(
        '\n\t%s Hokuyo was seen %5.3f seconds ago: %g m',
        h.name,t_now - h.t_last,h.mid)
      end
      os.execute('clear')
      print(debug_str)
      t0 = t_now
      main_cnt = 0
    end
    coroutine.yield()
  end
end
libHokuyo.service( hokuyos, main )

