---------------------------------
---- Camera sensor manager for Team THOR
---- (c) Stephen McGill, Yida Zhang 2013
-----------------------------------
dofile 'include.lua'
local Body       = require'Body'
local uvc        = require'uvc'
local carray     = require'carray'
local simple_ipc = require'simple_ipc'
local jpeg       = require'jpeg'
local udp        = require 'udp'
local unix       = require 'unix'


--[[
local use_wrist_cameras = true
-- Set up the UDP channel for broadcasting
local head_udp_img = udp.new_sender(UDP_IP, PORT_HCAMERA);
local left_udp_img = udp.new_sender(UDP_IP, PORT_LCAMERA);
local right_udp_img = udp.new_sender(UDP_IP, PORT_RCAMERA);
local head_quality = 50;
local head_width,head_height = 320, 240;
local head_frame_rate = 30
local hand_quality = 20;
local left_width, left_height = 320, 240;
local left_frame_rate = 30
local right_width, right_height = 320, 240;
local right_frame_rate = 30
--]]

local head_udp_img = udp.new_sender(Config.udp.UDP_IP, Config.udp.PORT_HCAMERA);
local left_udp_img = udp.new_sender(Config.udp.UDP_IP, Config.udp.PORT_LCAMERA);
local right_udp_img = udp.new_sender(Config.udp.UDP_IP, Config.udp.PORT_RCAMERA);
local use_wrist_cameras = Config.camera.use_wrist_camera;

local head_quality = Config.camera.head_quality;
local head_width,head_height = 
  Config.camera.head_resolution[1],Config.camera.head_resolution[2]; 
local head_frame_rate = Config.camera.head_fps
local hand_quality = Config.camera.hand_quality;
local left_width, left_height = 
  Config.camera.hand_resolution[1],Config.camera.hand_resolution[2]; 
local left_frame_rate = Config.camera.hand_fps;
local right_width, right_height = 
  Config.camera.hand_resolution[1],Config.camera.hand_resolution[2]; 
local right_frame_rate = Config.camera.hand_fps;

local head_dev = Config.camera.head_device
local left_dev = Config.camera.left_device
local right_dev = Config.camera.right_device

-- Open each of the cameras
unix.usleep(1e5)
local head_camera = 
  uvc.init(head_dev, head_width, head_height, 'yuyv',1,head_frame_rate)
-- Create the callback functions
local head_camera_poll = {}
head_camera_poll.socket_handle = head_camera:descriptor()
head_camera_poll.ts = Body.get_time()
head_camera_poll.count = 0
head_camera_poll.callback = function()
  local t_img = Body.get_time()
  local head_img, head_img_sz = head_camera:get_image()

  if head_camera_poll.count%head_frame_rate==0 then
    local t_now = Body.get_time()
    local t_diff = t_now - head_camera_poll.ts
    print(string.format('Head | %5.2f FPS', head_camera_poll.count/t_diff))
    head_camera_poll.count = 0
    head_camera_poll.ts = t_now
  end
  head_camera_poll.count = head_camera_poll.count + 1

  -- Send over UDP

  jpeg.set_quality( head_quality );
  local head_jimg = jpeg.compress_yuyv(head_img,head_width,head_height)
  local udp_ret = head_udp_img:send( head_jimg,#head_jimg )
  if udp_ret~=#head_jimg then
    print('Bad camera send',udp_ret)
  end
end

--[[


local left_camera = 
  uvc.init(left_dev, left_width, left_height, 'mjpeg',1,left_frame_rate)
-- Create the callback functions
local left_camera_poll = {}
left_camera_poll.socket_handle = left_camera:descriptor()
left_camera_poll.ts = Body.get_time()
left_camera_poll.count = 0
left_camera_poll.callback = function()
  local t_img = Body.get_time()
  local left_img, left_img_sz = left_camera:get_image()

  if left_camera_poll.count%left_frame_rate==0 then
    local t_now = Body.get_time()
    local t_diff = t_now - left_camera_poll.ts
    print(string.format('Left | %5.2f FPS', left_camera_poll.count/t_diff))
    left_camera_poll.count = 0
    left_camera_poll.ts = t_now
  end
  left_camera_poll.count = left_camera_poll.count + 1

  -- Send over UDP

  local udp_ret = left_udp_img:send( left_img, left_img_sz )
  if udp_ret~=left_img_sz then
    print('Bad camera send',udp_ret)
  end
end


local right_camera = 
  uvc.init(right_dev, right_width, right_height, 'mjpeg',1,right_frame_rate)
-- Create the callback functions
local right_camera_poll = {}
right_camera_poll.socket_handle = right_camera:descriptor()
right_camera_poll.ts = Body.get_time()
right_camera_poll.count = 0
right_camera_poll.callback = function()
  local t_img = Body.get_time()
  local right_img, right_img_sz = right_camera:get_image()

  if right_camera_poll.count%right_frame_rate==0 then
    local t_now = Body.get_time()
    local t_diff = t_now - right_camera_poll.ts
    print(string.format('Right | %5.2f FPS', right_camera_poll.count/t_diff))
    right_camera_poll.count = 0
    right_camera_poll.ts = t_now
  end
  right_camera_poll.count = right_camera_poll.count + 1

  -- Send over UDP

  local udp_ret = right_udp_img:send( right_img, right_img_sz )
  if udp_ret~=right_img_sz then
    print('Bad camera send',udp_ret)
  end
end
--]]

--[[

---------------
-- Wrist
-- Open each of the cameras
unix.usleep(1e5)
local left_camera = 
  uvc.init( left_dev, left_width, left_height, 'yuyv',1,left_frame_rate)
-- Create the callback functions
local left_camera_poll = {}
left_camera_poll.socket_handle = left_camera:descriptor()
left_camera_poll.ts = Body.get_time()
left_camera_poll.count = 0
left_camera_poll.callback = function()
  local t_img = Body.get_time()
  local img, img_sz = left_camera:get_image()

  if left_camera_poll.count%left_frame_rate==0 then
    local t_now = Body.get_time()
    local t_diff = t_now - left_camera_poll.ts
    print(string.format('Left | %5.2f FPS', left_camera_poll.count/t_diff))
    left_camera_poll.count = 0
    left_camera_poll.ts = t_now
  end
  left_camera_poll.count = left_camera_poll.count + 1

  -- Send over UDP
  jpeg.set_quality( hand_quality );
  local jimg = jpeg.compress_yuyv( img, left_width, left_height )
  local udp_ret = left_udp_img:send( jimg,#jimg )
  if udp_ret~=#jimg then
    print('Bad left camera send',udp_ret)
  end
end
---------------

-- Open each of the cameras
unix.usleep(1e5)
local right_camera =
  uvc.init( right_dev, right_width, right_height, 'yuyv',1,right_frame_rate)
-- Create the callback functions
local right_camera_poll = {}
right_camera_poll.socket_handle = right_camera:descriptor()
right_camera_poll.ts = Body.get_time()
right_camera_poll.count = 0
right_camera_poll.callback = function()
  local t_img = Body.get_time()
  local img, img_sz = right_camera:get_image()
  if right_camera_poll.count%right_frame_rate==0 then
    local t_now = Body.get_time()
    local t_diff = t_now - right_camera_poll.ts
    print(string.format('Right | %5.2f FPS', right_camera_poll.count/t_diff))
    right_camera_poll.count = 0
    right_camera_poll.ts = t_now
  end
  right_camera_poll.count = right_camera_poll.count + 1

  -- Send over UDP
  jpeg.set_quality( hand_quality );
  local jimg = jpeg.compress_yuyv( img, right_width, right_height )
  local udp_ret = right_udp_img:send( jimg,#jimg )
  if udp_ret~=#jimg then
    print('Bad right camera send',udp_ret)
  end
end
---------------

--]]




-- Poll multiple sockets
local wait_channels = { head_camera_poll, left_camera_poll, right_camera_poll }
--local wait_channels = { head_camera_poll};
local channel_poll = simple_ipc.wait_on_channels( wait_channels );
local channel_timeout = 1e5
-- Only the callback will be made: no timeout
--channel_poll:start()
local cnt = 0;
local t_last = Body.get_time()
local t_debug = 1
while true do
  local npoll = channel_poll:poll(channel_timeout)
  local t = Body.get_time()
  cnt = cnt+1;
  if t-t_last>t_debug then
    local msg = string.format("%.2f FPS", cnt/t_debug);
    print("Camera Wizard | "..msg,npoll)
    t_last = t;
    cnt = 0;
  end
end
