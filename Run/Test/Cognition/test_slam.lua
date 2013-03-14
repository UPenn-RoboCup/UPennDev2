dofile('../../include.lua')

-- Require the right modules
local Sensors = require 'sensors/Config_Sensors'
local simple_ipc = require 'simple_ipc'
local libSlam = require 'libSlam'
local mp = require 'MessagePack'
require 'unix'
require 'cjpeg'
require 'cutil'


-- Initialize the map
local omap = libSlam.OMAP.data
omap:fill(127) -- Uncertain
print('Map size:',libSlam.MAPS.sizex,libSlam.MAPS.sizey)

-- Setup IPC
local omap_channel = simple_ipc.setup_publisher('omap');

local lidar_channel = simple_ipc.setup_subscriber('lidar');
local lidar_callback = function()
  -- Receive ipc sensor payload
  local lidar_data, has_more = lidar_channel:receive()
  local lidar_tbl = mp.unpack( lidar_data );
  Sensors.LIDAR0.timestamp = lidar_tbl.t;
  -- Place in the torch storage
  -- TODO: this assumes is it contiguous
  cutil.string2userdata( Sensors.LIDAR0.ranges:storage():pointer(), lidar_tbl.ranges)
  libSlam.processL0()
  --libSlam.OMAP.timestamp
  local omap_s_ptr = omap:storage():pointer()
  local jomap = cjpeg.compress( omap_s_ptr, libSlam.MAPS.sizex, libSlam.MAPS.sizey,1 )
  omap_channel:send( jomap );
end
lidar_channel.callback = lidar_callback

local imu_channel = simple_ipc.setup_subscriber('imu');
imu_channel.callback = function()
	local nbytes, has_more = imu_channel:receive(imu_buf, imu_buf_sz)
	local offset, decoded = mp.unpack( ffi.string(imu_buf,nbytes) )
end

-- Poll multiple sockets
local wait_channels = {lidar_channel, imu_channel}
local channel_poll = simple_ipc.wait_on_channels( wait_channels );

-- Start the timing
local t = unix.time();
local t_last = t;
local map_rate = 15; --15Hz
local map_t = 1/(map_rate)
local channel_timeout = 2*map_t*1e3; -- milliseconds, or just wait (-1)
local t_last_lidar = Sensors.LIDAR0.timestamp;


-- This loops itself
-- This just does the callbacks
channel_poll:start()

--[[
while true do
  --this will go through the loop and send the callbacks
  --Have not tested this much...
  --channel_poll:poll(channel_timeout) 

  -- Send the map at set intervals
  t = unix.time()
  if t-t_last>map_t then
    print('hi!')
    t_last = t;
  end
end
--]]
