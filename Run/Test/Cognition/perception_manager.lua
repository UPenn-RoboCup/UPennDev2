-- Set the path for the libraries
dofile('../../include.lua')
-- Set the Debugging mode
local debug = true;

-- Libraries
local simple_ipc = require 'simple_ipc'
local carray = require 'carray'
local Octomap = require'Octomap'
local torch = require'torch'
torch.Tensor = torch.DoubleTensor
local libLaser = require 'libLaser'

-- Global vars
require 'unix'

-- IPC channels
local omap_channel = simple_ipc.new_publisher('omap')
local oct_channel  = simple_ipc.new_publisher('oct')

local lidar_channel = simple_ipc.new_subscriber('lidar')
lidar_channel.callback = function()
  local ts, has_more = lidar_channel:receive();
  if not has_more then
    print( "LIDAR | Bad ts", type(ts) )
    return
  end
  local ranges_str, has_more = lidar_channel:receive();
  local lidar_ts = tonumber(ts);
  local ranges_f = carray.double( ranges_str );

  -- TODO: Do not use a silly element-by-element copy
  local ranges = torch.Tensor( 1081 )
  for i=1,#ranges_f do
    ranges[i] = ranges_f[i]
  end
  
  -- TODO: Use for slam as well
  libLaser.ranges2xyz(ranges,0,0,0)
  --Octomap.add_scan( libLaser.points_xyz )
  local ret = omap_channel:send("done laser")
  if ret then
    print("LIDAR | OMAP update", unix.time() )
  else
    print("Bad omap channel!")
  end
  
  ret = oct_channel:send("done oct")
  if ret then
    print("LIDAR | Oct Tree update", unix.time() )
  else
    print("Bad oct channel!")
  end
  
end

-- Poll multiple sockets
local wait_channels = {lidar_channel}
local channel_poll = simple_ipc.wait_on_channels( wait_channels )
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
    print(npoll,"Perception Manager | "..msg)
    t_last = t;
    cnt = 0;
  end
end
