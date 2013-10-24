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
-- Track the cameras
local wait_channels = {}
local cameras = {}

-- Use wired
local operator = Config.net.operator.wired

local function setup_camera(cam,name)
  -- Get the config values
  local dev     = cam.head.device
  local fps     = cam.head.fps
  local quality = cam.head.quality
  local width, height = unpack(Config.camera.head.resolution)
  -- Save the metadata
  local meta = {}
  meta.t = unix.time()
  meta.name = name..'_camera'
  meta.width = width
  meta.height = height
  
  -- Open the camera
  local dev = uvc.init(dev, width, height, 'yuyv', 1, fps)
  -- Open the local channel for logging
  local cam_pub_ch = simple_ipc.new_publisher(meta.name)
  -- Open the unreliable network channel
  local camera_udp_ch = udp.new_sender(operator, net)
  
  -- Export
  local camera = {}
  camera.meta = meta
  camera.dev  = dev
  camera.pub  = cam_pub_ch
  camera.udp  = camera_udp_ch
  return camera
end

-- Open each of the cameras
for name,cam in pairs(Config.cameras) do
  local net = Config.net.camera[name]
  -- Open the camera
  local camera = setup_camera(cam,name,net)
  -- Create the camera callback function
  local camera_poll = {}
  camera_poll.socket_handle = camera:descriptor()
  camera_poll.ts = unix.time()
  camera_poll.count = 0
  camera_poll.callback = function()
    local img, head_img_sz = camera:get_image()
    -- Update the metadata
    camera.meta.t = unix.time()
    camera.meta.c = 'jpeg'
    camera.meta.count = camera.meta.count + 1
    local metapack = mp.pack(meta)
    -- Compress the image
    jpeg.set_quality( quality )
    local jimg = jpeg.compress_yuyv(img,width,height)
    -- Send over UDP
    local udp_ret, err = camera_udp_ch:send( metapack..jimg )
    if err then print(camera.meta.name,'udp error',err) end
    -- Send to the local channel for logging
    cam_pub_ch:send( {mp.pack(meta), jimg} )
  end
  -- Keep track of this camera
  table.insert(wait_channels,camera_poll)
  table.insert(cameras,camera)
end

-- Close the camera properly upon Ctrl-C
local signal = require 'signal'
local function shutdown()
  print'Shutting down the Cameras...'
  for name,cam in pairs(cameras) do
    cam.dev:close()
    print('Closed camera',name)
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

while true do
  local npoll = channel_poll:poll(channel_timeout)
  local t = unix.time()
  local t_diff = t-t_debug
  if t_diff>DEBUG_INTERVAL then
    t_debug = t
    print'debug...'
  end
end
