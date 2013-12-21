---------------------------------
---- Camera sensor manager for Team THOR
---- (c) Stephen McGill, Yida Zhang 2013
-----------------------------------
dofile 'include.lua'
local uvc        = require'uvc'
local carray     = require'carray'
local simple_ipc = require'simple_ipc'
local jpeg       = require'jpeg'
local udp        = require'udp'
local mp         = require'msgpack'
local unix       = require'unix'
local util       = require'util'
require'vcm'
-- Track the cameras
local wait_channels = {}
local cameras = {}

-- Use wired
local operator = Config.net.operator.wired
local logging = false

local function setup_camera(cam,name,udp_port,tcp_port)
  -- Get the config values
  local dev    = cam.device
  local fps    = cam.fps
  local width, height = unpack(cam.resolution)
  local format = cam.format
  -- Save the metadata
  local meta = {}
  meta.t = 0
  meta.count = 0
  meta.name = name..'_camera'
  meta.width = width
  meta.height = height
  
  -- Open the camera
  local dev = uvc.init(dev, width, height, format, 1, fps)
  -- Open the local channel for logging
  local cam_pub_ch = simple_ipc.new_publisher(meta.name)
  -- Open the unreliable network channel
  local camera_udp_ch = udp.new_sender(operator, udp_port)
  -- Open the reliable network channel
  local camera_tcp_ch = simple_ipc.new_publisher(tcp_port,false,'*')
  
  -- Export
  local camera = {}
  camera.meta = meta
  camera.dev  = dev
  camera.pub  = cam_pub_ch
  camera.udp  = camera_udp_ch
  camera.tcp  = camera_tcp_ch
  camera.format = format
  return camera
end

local socket_handle_lut = {}

-- Open each of the cameras
--for name,cam in pairs(Config.camera) do
for name,cam in pairs{forehead=Config.camera.forehead} do

  -- Debug
  print(util.color('Setting up','green'),name,cam.device)

  local udp_net = Config.net.camera[name]
  local tcp_net = Config.net.reliable_camera[name]
  -- Open the camera
  local camera = setup_camera(cam,name,udp_net,tcp_net)
  -- Create the camera callback function
  local camera_poll = {}
  camera_poll.socket_handle = camera.dev:descriptor()
  camera_poll.count = 0
  camera_poll.ts = 0
  -- Net settings get/set
  camera_poll.get_net = vcm['get_'..name..'_camera_net']
  camera_poll.set_net = vcm['set_'..name..'_camera_net']
  camera_poll.camera = camera
  -- Frame callback
  camera_poll.callback = function(sh)
    -- Identify which camera
    local camera_poll = wait_channels.lut[sh]
    local t = unix.time()
    camera_poll.ts = t
    local camera = camera_poll.camera
    -- Grab the image
    --local img, img_sz = camera.dev:get_rotated_image()
    local img, img_sz = camera.dev:get_image()
    if type(img)=='number' then os.exit() end
    -- Grab the net settings
    local net_settings = camera_poll.get_net()
    local stream, method, quality, interval = unpack(net_settings)

    -- If not sending, then no computation
    if stream==0 then return end
    
    -- Streaming interval check
    if stream==2 or stream==4 then
      -- If we have not waited enough time
      local t_diff_send = t - camera.meta.t
      if t_diff_send<interval then return end
    end
    
    -- Compress the image (ignore the 'method' for now - just jpeg)
    local c_img
    if camera.format=='yuyv' then
      -- yuyv
      jpeg.set_quality( quality )
      c_img = jpeg.compress_yuyv(img,camera.meta.width,camera.meta.height)
    else
      -- mjpeg
      c_img = tostring(carray.char(img,img_sz))
    end

    -- Update the metadata
    camera.meta.t = t
    camera.meta.c = 'jpeg'
    camera.meta.count = camera.meta.count + 1
    camera.meta.sz = #c_img
    local metapack = mp.pack(camera.meta)
    
    -- Send to the logger
    if logging then camera.pub:send( {metapack, c_img} ) end

    -- Send over the network
    if stream==1 or stream==2 then
      -- Send over UDP
      local udp_ret, err = camera.udp:send( metapack..c_img )
      if err then print(camera.meta.name,'udp error',err) end
    elseif stream==3 or stream==4 then
      -- Send over TCP
      local ret = camera.tcp:send{metapack,c_img}
    end
    
    -- Turn off single frame
    if stream==1 or stream==3 then
      net_settings[1] = 0
      camera_poll.set_net(net_settings)
    end
    
    -- Simple debug for the time being
    --print(camera.meta.name,'Sent frame!',t)
    
  end

  -- Debug
  print(util.color('=======','yellow'))
  util.ptable(camera.meta)

  -- Keep track of this camera
  table.insert(wait_channels,camera_poll)
  table.insert(cameras,camera)

  unix.usleep(1e3)

end

-- Close the camera properly upon Ctrl-C
local signal = require 'signal'
local function shutdown()
  print'Shutting down the Cameras...'
  for _,cam in ipairs(cameras) do
    cam.dev:close()
    print('Closed camera',cam.meta.name)
  end
  os.exit()
end
signal.signal("SIGINT", shutdown)
signal.signal("SIGTERM", shutdown)

-- Poll multiple sockets
assert(#cameras>0,'No cameras available!')
local channel_poll = simple_ipc.wait_on_channels( wait_channels );
local channel_timeout = 1e3 --ms, means 1 sec timeout = 1e3

-- Debugging
local t_debug = unix.time()
local DEBUG_INTERVAL = 1

-- Begin to poll
while true do
  local npoll = channel_poll:poll(channel_timeout)
  local t = unix.time()
  -- Debug messages if desired
  local t_diff = t-t_debug
  if t_diff>DEBUG_INTERVAL then
    t_debug = t
    --print('Head Count')
  end
end
