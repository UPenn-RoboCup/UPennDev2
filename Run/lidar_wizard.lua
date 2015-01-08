#!/usr/bin/env luajit
-- Hokuyo LIDAR Wizard
-- Reads lidar scans and saves to shared memory
-- (c) Stephen McGill 2013, 2014
dofile'../include.lua'

local unix = unix or require'unix'
local libHokuyo  = require'libHokuyo'
local signal = require'signal'.signal
local get_time = unix.time
local mpack = require'msgpack'.pack
local color = require'util'.color
local si = require'simple_ipc'
local Body = require'Body'
require'wcm'

local cb = function(self, data)
	self.ch:send({
	mpack({
		t = get_time(),
		n = self.n,
		res = self.res,
		angle = Body.get_lidar_position(),
    rpy = Body.get_rpy(), 
    pose = wcm.get_robot_odometry()
	}),
	data})
end

-- Setup the Hokuyos array
local hokuyos = {}

-- Initialize the Hokuyos
--local h0 = libHokuyo.new_hokuyo('/dev/ttyACM0')
--local h0 = libHokuyo.new_hokuyo('/dev/cu.usbmodem1411',nil,9600)

local h0
if HOSTNAME=='teddy' then
	h0 = libHokuyo.new_hokuyo(10)
elseif HOSTNAME == 'alvin' or true then
	h0 = libHokuyo.new_hokuyo(11)
else
	print('WRONG HOST NAME !!')
end

h0.name = 'front'
h0.ch = si.new_publisher'lidar0'
h0.callback = cb
table.insert(hokuyos, h0)

-- Ensure that we shutdown the devices properly
local function shutdown()
  print'Shutting down the Hokuyos...'
  for i,h in ipairs(hokuyos) do
    h:stream_off()
    h:close()
    print('Closed Hokuyo',i)
  end
  os.exit()
end
signal("SIGINT", shutdown)
signal("SIGTERM", shutdown)

-- Begin to service
os.execute('clear')
assert(#hokuyos>0, "No hokuyos detected!")
print( color('Servicing '..#hokuyos..' Hokuyos','green') )

local main = function()
  local main_cnt = 0
  local t0 = get_time()
  while true do
    main_cnt = main_cnt + 1
    local t_now = get_time()
    local t_diff = t_now - t0
    if t_diff>1 then
      local debug_str = string.format('\nMain loop: %7.2f Hz',main_cnt/t_diff)
      debug_str = color(debug_str,'yellow')
      for i,h in ipairs(hokuyos) do
        debug_str = debug_str..string.format(
        '\n\t%s Hokuyo:\t%5.1fHz\t%4.1f ms ago',h.name, 1/h.t_diff, (t_now-h.t_last)*1000)
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
