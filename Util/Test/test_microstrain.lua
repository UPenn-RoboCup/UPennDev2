dofile'../../include.lua'
local Body       = require'Body'
local signal     = require'signal'
local carray     = require'carray'
local mp         = require'msgpack'
local util       = require'util'
local simple_ipc = require'simple_ipc'
local libMicrostrain  = require'libMicrostrain'


local imu = libMicrostrain.new_microstrain(
  '/dev/cu.usbmodem1421', 115200 )

if not imu then
  print('No imu present!')
  os.exit()
end

util.ptable(imu)
--[[
print('READING')
ret = unix.read(imu.fd)
print('Ret',ret)

ping = string.char(0x75 ,0x65 ,0x01 ,0x02 ,0x02 ,0x01 ,0xE0 ,0xC6)
print(type(ping))

ret = unix.write(imu.fd,ping)
--unix.usleep(1e4)

local ret_fd = unix.select( {imu.fd} )

print('READING',ret_fd,imu.fd)
ret = unix.read(imu.fd)
stuff = {ret:byte(1,-1)}
--]]
--[[
for i,b in ipairs(stuff) do
  print( string.format('%d: %X',i,b) )
end
--]]

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

