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
  local dev     = cam.device
  local fps     = cam.fps
  local quality = cam.quality
  local width, height = unpack(cam.resolution)
  local format = cam.format
  -- Save the metadata
  local meta = {}
  meta.t = unix.time()
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
  camera.quality = cam.quality
  camera.format = format
  return camera
end

-- Open each of the cameras
for name,cam in pairs(Config.camera) do

  -- Debug
  print(util.color('Setting up','green'),name)

  local udp_net = Config.net.camera[name]
  local tcp_net = Config.net.reliable_camera[name]
  -- Open the camera
  local camera = setup_camera(cam,name,udp_net,tcp_net)
  -- Create the camera callback function
  local camera_poll = {}
  camera_poll.socket_handle = camera.dev:descriptor()
  camera_poll.ts = unix.time()
  camera_poll.count = 0
  -- Net settings get/set
  local get_net = vcm['get_'..name..'_camera_net']
  local set_net = vcm['set_'..name..'_camera_net']
  camera_poll.get_net = vcm['get_'..name..'_camera_net']
  camera_poll.set_net = vcm['set_'..name..'_camera_net']
  -- Frame callback
  camera_poll.callback = function()
    -- Grab the net settings to see if we should actually send this frame
    local net_settings = vcm.get_head_camera_net()
    local stream, method, quality = unpack(net_settings)
    camera.quality = quality
    local img, head_img_sz = camera.dev:get_image()

    -- No streaming, so no computation
    if stream==0 and not logging then return end
    
    -- Compress the image (ignore the 'method' for now - just jpeg)
    local c_img
    if camera.format=='yuyv' then
      -- yuyv
      jpeg.set_quality( camera.quality )
      c_img = jpeg.compress_yuyv(img,camera.meta.width,camera.meta.height)
    else
      -- mjpeg
      c_img = tostring(carray.char(img,head_img_sz))
    end

    -- Update the metadata
    camera.meta.t = unix.time()
    camera.meta.c = 'jpeg'
    camera.meta.count = camera.meta.count + 1
    camera.meta.sz = #c_img
    local metapack = mp.pack(camera.meta)
    
    -- Send for logger
    if logging then camera.pub:send( {metapack, c_img} ) end
    
    -- If no network streaming, then return
    if stream==0 then return end

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
      vcm['set_'..camera.meta.name..'_net'](net_settings)
    end
    
  end

  -- Debug
  print(util.color('=======','yellow'))
  util.ptable(camera.meta)

  -- Keep track of this camera
  table.insert(wait_channels,camera_poll)
  table.insert(cameras,camera)
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
