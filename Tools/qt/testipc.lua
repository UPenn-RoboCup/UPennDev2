local pwd = os.getenv('PWD')
package.cpath = pwd..'/../../Player/Lib/?.so;'..package.cpath
package.path = pwd..'/../../Player/Util/ffi/?.lua;'..package.path
package.path = pwd..'/../../Player/Util/?.lua;'..package.path

local simple_ipc = require 'simple_ipc'
local mp = require 'MessagePack'
local imu_channel = simple_ipc.setup_subscriber('img');
local img_channel = simple_ipc.new_publisher('img2');
local util = require 'util'
local unix = require 'unix'

imu_channel.callback = function()
  local imu_data, has_more = imu_channel:receive();
--  print(mp.unpack(imu_data)..' '..unix.time())
  t = mp.unpack(imu_data)
  util.ptable(t)
--  print(unix.time())
  print(has_more)
  while (has_more) do
    imu_data, has_more = imu_channel:receive();
--    print('1 '..unix.time())
  end
--  print(mp.unpack(imu_data)..' '..unix.time())
end
local wait_channels = { imu_channel }
local channel_poll = simple_ipc.wait_on_channels( wait_channels );

local channel_timeout = 1e3;
--channel_timeout = 0;
while (1) do
--  channel_poll:poll(channel_timeout)
  local imu_data, has_more = imu_channel:receive();
  img_channel:send('a'..imu_data);
  print(#imu_data..' '..unix.time())
--  print(mp.unpack(imu_data)..' '..unix.time())
--  print(unix.time())
end
