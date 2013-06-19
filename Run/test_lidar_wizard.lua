-----------------------------------------------------------------
-- Combined Lidar manager for Team THOR
-- Reads and sends raw lidar data
-- As well as accumulate them as a map
-- and send to UDP
-- (c) Stephen McGill, Seung Joon Yi, 2013
---------------------------------

dofile'include.lua'

-- Use both lidars?
--use_second_lidar = false;
use_second_lidar = true;
lidar_0_head = true; --to detect the lidar type
--lidar_0_head = false; --to detect the lidar type

-- Libraries
local simple_ipc = require'simple_ipc'
local carray = require'carray'
local libHokuyo = require'libHokuyo'
local signal = require'signal'
require 'unix'

local serial0 = "00907258" -- Head
local serial1 = "00805676" -- Chest

local device0 = "/dev/ttyACM0"
local device1 = "/dev/ttyACM1"

if OPERATING_SYSTEM=='darwin' then
  device0 = "/dev/cu.usbmodem1421"
  device1 = "/dev/cu.usbmodem1422"
end

--Lidar variables
local hokuyo_0 = nil
local hokuyo_1 = nil
local lidar_channel_0 = nil
local lidar_channel_1 = nil
local hokuyo0_poll = {}
local hokuyo1_poll = {}
local wait_channels = {}
lidar_channel_0 = simple_ipc.new_publisher('lidar0') --head lidar
lidar_channel_1 = simple_ipc.new_publisher('lidar1') --chest lidar

--Callback functions
local function hokuyo0_callback()
	local data = hokuyo_0:get_scan()
	local t = unix.time()
	if not data then
		print('BAD DATA')
		return
	end
	print('Got ',#data)
	local ranges = carray.float( data, 1081 )
	print('Midrange jumper',ranges[540])
end

local function hokuyo1_callback()
  local data = hokuyo_1:get_scan()
  local t = unix.time()
  if not data then
		print('BAD DATA')
		return
	end
  local senddata = {tostring(t),data}
	local ranges = carray.float( data, 1081 )
	print('Midrange jumper chest',ranges[540])
  if lidar_0_head then
		send_lidar_data(senddata,1)
  else
		send_lidar_data(senddata,0)
  end 
end

-- Send to ZMQ channels
if true then
  hokuyo_0 = libHokuyo.open( device0 )--,serial0)
  hokuyo0_poll.socket_handle = hokuyo_0.fd
  hokuyo0_poll.callback = hokuyo0_callback;
  hokuyo_0:stream_on()
  table.insert( wait_channels, hokuyo0_poll )
end

if use_second_lidar then
  hokuyo_1 = libHokuyo.open( device1 )--,serial2)
  hokuyo1_poll.socket_handle = hokuyo_1.fd
  hokuyo1_poll.callback = hokuyo1_callback;
  hokuyo_1:stream_on()
  table.insert( wait_channels, hokuyo1_poll )
end


--[[
  --TODO: check lidar serial here
	-- Implementation: just swap the references :)
  if hokuyo1_serial==serial1 then
    lidar_0_head = true; --to detect the lidar type
  else
    lidar_0_head = false; --to detect the lidar type
  end
--]]


-- Ensure that we shutdown the devices properly
function shutdown()
  print'Shutting down the Hokuyos...'
  hokuyo_0:stream_off()
  hokuyo_0:close()
  print'Closed Hokuyo 0'
  if use_second_lidar then
    hokuyo_1:stream_off()
    hokuyo_1:close()
    print'Closed Hokuyo 1'
  end
  error()
end

-- Ensure a proper shutdown
signal.signal("SIGINT", shutdown)
signal.signal("SIGTERM", shutdown)

-- Polling with zeromq
local channel_poll = simple_ipc.wait_on_channels( wait_channels )
channel_poll:start()
--[[
local channel_timeout = 100;
while true do
  local npoll = channel_poll:poll(channel_timeout)
end
--]]
