dofile('../../Run/include.lua')

local simple_ipc = require 'simple_ipc'
local mp = require 'MessagePack'
local imu_channel = simple_ipc.new_subscriber('test');
local util = require 'util'
local unix = require 'unix'

imu_channel.callback = function()
  local imu_data, has_more = imu_channel:receive();
  print(#imu_data..' '..unix.time())
--  print(mp.unpack(imu_data)..' '..unix.time()..' '..has_more)
end
local wait_channels = { imu_channel }
local channel_poll = simple_ipc.wait_on_channels( wait_channels );

local channel_timeout = 1e3;
--channel_timeout = 0;
while (1) do
--  channel_poll:poll(channel_timeout)
  local imu_data, has_more = imu_channel:receive();
  print(#imu_data..' '..unix.time())
--  print(mp.unpack(imu_data)..' '..unix.time())
--  print(unix.time())
end
