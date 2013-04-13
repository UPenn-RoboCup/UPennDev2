-- Set the path for the libraries
dofile('../../include.lua')
-- Set the Debugging mode
local debug = true;

-- Libraries
local simple_ipc = require 'simple_ipc'
require 'unix'
require 'cjpeg'

-- Setup IPC Channels
local omap_channel   = simple_ipc.new_subscriber('omap');
local oct_channel    = simple_ipc.new_subscriber('oct');
local hmi_channel    = simple_ipc.new_subscriber('hmi');

-- Replan based on occupancy map
omap_channel.callback = function()
  local omap_data, has_more = omap_channel:receive()
  -- TODO: Get the timestamp
  print('OMAP | ', 'Replanned waypoints!')
end

-- Replan based on Oct Tree
oct_channel.callback = function()
  local oct_data, has_more = oct_channel:receive()
  -- TODO: Get the timestamp
  print('Oct | ', 'Replanned manipulation targets!')
end

-- Replan based on HMI
hmi_channel.callback = function()
  local hmi_data, has_more = hmi_channel:receive()
  -- TODO: Get the timestamp
  print('HMI | ', 'Replanned waypoints!')
end

-- Poll multiple sockets
local wait_channels = {omap_channel, oct_channel, hmi_channel}
local channel_poll = simple_ipc.wait_on_channels( wait_channels );
local channel_timeout = 100; -- 100ms timeout

-- Start the timing
local t = unix.time();
local t_last = t;
local t_debug = 1; -- Print debug output every second

-- No debugging messages
-- Only the callback will be made
if not debug then
  channel_poll:start()
end

local cnt = 0;
while true do
  local npoll = channel_poll:poll(channel_timeout)
  local t = unix.time()
  cnt = cnt+1;
  if t-t_last>t_debug then
    local msg = string.format("%.2f FPS", cnt/t_debug);
    print("Behavior Manager | "..msg)
    t_last = t;
    cnt = 0;
  end
end