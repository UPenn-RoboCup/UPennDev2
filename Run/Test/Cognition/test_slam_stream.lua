dofile('../../include.lua')

-- Require the right modules
local Sensors = require 'sensors/Config_Sensors'
local simple_ipc = require 'simple_ipc'
local libSlam = require 'libSlam'
local mp = require 'MessagePack'
require 'unix'
require 'cjpeg'
require 'cutil'

-- Setup IPC
local lidar_channel = simple_ipc.new_subscriber('lidar');
local imu_channel = simple_ipc.new_subscriber('arduimu');

local lidar_callback = function()
  -- Receive ipc sensor payload
  local lidar_data, has_more = lidar_channel:receive()
  local lidar_tbl = mp.unpack( lidar_data );
  Sensors.LIDAR0.timestamp = lidar_tbl.t;
  -- Place in the torch storage
  -- TODO: this assumes is it contiguous
  cutil.string2userdata( Sensors.LIDAR0.ranges:storage():pointer(), lidar_tbl.ranges)
  collectgarbage("stop")
  libSlam.processL0()
  -- Cannot restart... it is a problem!
  --collectgarbage("restart")
end
lidar_channel.callback = lidar_callback

local imu_callback = function()
  local imu_data, has_more = imu_channel:receive()
  local imu_tbl = mp.unpack( imu_data )
  --[[
  for i,v in pairs(imu_tbl) do
    print(i,v)
  end
  --]]
--  Sensors.IMU.data = imu_tbl;
  libSlam.processIMU( imu_tbl )
end
imu_channel.callback = imu_callback

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
--channel_poll:start()
require 'Comm'
Comm.init('192.168.123.255', 54321);

--while true do
for i=1,15 do
  --this will go through the loop and send the callbacks
  --Have not tested this much...
  channel_poll:poll(channel_timeout) 

  -- Send the map at set intervals
  t = unix.time()
  if t-t_last>map_t then
    local omap_s_ptr = libSlam.OMAP.data:storage():pointer()
    local jomap = cjpeg.compress( omap_s_ptr, libSlam.MAPS.sizex, libSlam.MAPS.sizey,1 )
    print('Sending map',i)
    Comm.send( jomap );
    t_last = t;
  end
end
