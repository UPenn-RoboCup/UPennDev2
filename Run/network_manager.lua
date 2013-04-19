-- Set the path for the libraries
dofile('include.lua')
-- Set the Debugging mode
local debugging = true;

-- Libraries
local simple_ipc = require 'simple_ipc'
require 'unix'
require 'cjpeg'

-- Set up the UDP channel for broadcasting
local udp = require 'udp'
local hmi_udp_img_snd = udp.new_sender('127.0.0.1',54321)
local hmi_udp_omap_snd = udp.new_sender('127.0.0.1',54322)
local hmi_udp_oct_snd = udp.new_sender('127.0.0.1',54323)
-- UDP receiver with file descriptor
local hmi_udp_recv = udp.new_receiver(54320)
local hmi_udp_recv_poll = {}
hmi_udp_recv_poll.socket_handle = hmi_udp_recv
hmi_udp_recv_poll.callback = function()
	while udp.size()>0 do
		local hmi_data = udp.receive()
		print('\tLOCAL | Received',data)
	end
	print('HMI RECV DATA!!!!')
end

-- Setup IPC Channels
-- Publishers
-- TODO: How to receive HMI commands over the network?
local hmi_channel    = simple_ipc.new_publisher('hmi');
-- Subscribers
local omap_channel   = simple_ipc.new_subscriber('omap');
local camera_channel = simple_ipc.new_subscriber('img');
local oct_channel    = simple_ipc.new_subscriber('oct');

-- Send the Occupmancy Map
-- JPEG compressed
omap_channel.callback = function()
  local omap_data, has_more = omap_channel:receive()
  local jimg = cjpeg.compress( omap_data );
  local nsent = udp.send( hmi_udp_sender, omap_data, #omap_data );
  print("OMAP | Sent ", nsent, #omap_data )
end

-- Send image over UDP
-- JPEG compress it
camera_channel.callback = function()
  local camera_ts, has_more = camera_channel:receive()
  -- TODO: Get timestamp
	if not has_more then
		print( 'Bad Camera | ', type(camera_ts), type(has_more) )
		return;
	end
  local camera_data, has_more = camera_channel:receive()
  local jimg = cjpeg.compress( camera_data, 640, 480 );
  local nsent = udp.send( hmi_udp_img_snd, jimg, #jimg );
	
	local debug_msg = string.format('Camera (%.2f) | ',camera_ts)
	if nsent==#jimg then
  	print(debug_msg.."Sent image!",camera_ts)
	else
		print(debug_msg.."Error sending image!")
	end
end

-- Send the Oct Tree data over UDP
oct_channel.callback = function()
  local oct_data, has_more = oct_channel:receive()
  local nsent = udp.send( hmi_udp_oct_snd, oct_data, #oct_data );
  print("Oct | Sent ", nsent )
end

-- Poll multiple sockets
local wait_channels = { omap_channel, camera_channel, oct_channel, hmi_udp_recv_poll }
--local wait_channels = { omap_channel, camera_channel, oct_channel }
local channel_poll = simple_ipc.wait_on_channels( wait_channels );
local channel_timeout = 100; -- 100ms timeout

-- Start the timing
local t = unix.time();
local t_last = t;
local t_debug = 1; -- Print debug output every second

-- No debugging messages
-- Only the callback will be made
if not debugging then
  channel_poll:start()
end

local cnt = 0;
while true do
  local npoll = channel_poll:poll(channel_timeout)
  local t = unix.time()
  cnt = cnt+1;
  if t-t_last>t_debug then
    local msg = string.format("%.2f FPS", cnt/t_debug);
    print("Network Manager | "..msg)
    t_last = t;
    cnt = 0;
  end
end
