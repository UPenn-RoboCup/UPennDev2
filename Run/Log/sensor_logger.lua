-- Set the path for the libraries
dofile('../../include.lua')
-- Set the Debugging mode
local debug = true;

-- Libraries
local simple_ipc = require 'simple_ipc'

-- Global vars
require 'unix'

-- IPC channels
local lidar_channel  = simple_ipc.new_subscriber('lidar')
local camera_channel = simple_ipc.new_subscriber('img')

-- Callbacks
lidar_channel.callback = function()
  local ts, has_more = lidar_channel:receive();
  if not has_more then
    print( "LIDAR | Bad ts", type(ts) )
    return
  end
  local ranges_str, has_more = lidar_channel:receive();
  
  -- TODO: Actually write the log
  
  print("LIDAR | Logged", ts )
end

camera_channel.callback = function()
  local camera_ts, has_more = camera_channel:receive()
  -- TODO: Get timestamp
  --[[
  if not has_more then
  print( 'Bad Camera | ', type(camera_ts), type(has_more) )
  return;
  end
  local camera_data, has_more = camera_channel:receive()
  local jimg = cjpeg.compress( camera_data );
  local nsent = Comm.send( jimg );
  --]]
  -- In the current form, gazebo does the compression for us
  -- This is probably not the best way to pipeline things at present..
  print("Camera | Logged" )
end

-- Poll multiple sockets
local wait_channels = {lidar_channel, camera_channel}
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
    print(npoll,"Sensor Logger | "..msg)
    t_last = t;
    cnt = 0;
  end
end
