dofile('../../include.lua')

-- Require the right modules
local simple_ipc = require 'simple_ipc'
require 'unix'
require 'cjpeg'
require 'cutil'

-- Setup IPC
local omap_channel = simple_ipc.setup_subscriber('omap');

local omap_callback = function()
  -- Receive ipc sensor payload
  omap_data, has_more = omap_channel:receive()
  --local omap_tbl = mp.unpack( lidar_data );
end
omap_channel.callback = omap_callback

-- Poll multiple sockets
local wait_channels = {omap_channel}
local channel_poll = simple_ipc.wait_on_channels( wait_channels );

-- Start the timing
local t = unix.time();
local t_last = t;
local channel_timeout = 2*1e3; -- milliseconds, or just wait (-1)

-- Save logs
filename = '/tmp/logs_omap/o_';
i = 0
while true do
  i = i+1
--for i=1,15 do
  channel_poll:poll(channel_timeout) 

  -- Send the map at set intervals
  t = unix.time()
  if omap_data then
    local jomap = cjpeg.compress( omap_data, 401, 401, 1 )
    if jomap then
      local myfile = string.format('%s%05d.jpeg',filename,i);
      print(myfile)
      local f = assert(io.open(myfile, 'w'))
      f:write(jomap);
      f:close()
    else
      print('bad jomap!')
    end
  end
end
