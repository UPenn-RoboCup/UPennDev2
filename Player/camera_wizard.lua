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
-- Track the cameras
local wait_channels = {}
local cameras = {}

-- Use wired
local operator = Config.net.operator.wired

local function setup_camera(cam,name,net)
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
  local camera_udp_ch = udp.new_sender(operator, net)
  
  -- Export
  local camera = {}
  camera.meta = meta
  camera.dev  = dev
  camera.pub  = cam_pub_ch
  camera.udp  = camera_udp_ch
  camera.quality = cam.quality
  camera.format = format
  return camera
end

-- Open each of the cameras
for name,cam in pairs(Config.camera) do

  -- Debug
  print(util.color('Setting up','green'),name)

  local net = Config.net.camera[name]
  -- Open the camera
  local camera = setup_camera(cam,name,net)
  -- Create the camera callback function
  local camera_poll = {}
  camera_poll.socket_handle = camera.dev:descriptor()
  camera_poll.ts = unix.time()
  camera_poll.count = 0
  camera_poll.callback = function()
    local img, head_img_sz = camera.dev:get_image()
    -- Compress the image
    local c_img
    if camera.format=='yuyv' then
      jpeg.set_quality( camera.quality )
      c_img = jpeg.compress_yuyv(img,camera.meta.width,camera.meta.height)
    else
      c_img = tostring(carray.char(img,head_img_sz))
      print('c_img',camera.meta.name,camera.format,#c_img)
    end

    -- Update the metadata
    camera.meta.t = unix.time()
    camera.meta.c = 'jpeg'
    camera.meta.count = camera.meta.count + 1
    camera.meta.sz = #c_img
    local metapack = mp.pack(camera.meta)

    -- Send over UDP
    local udp_ret, err = camera.udp:send( metapack..c_img )
    if err then print(camera.meta.name,'udp error',err) end
    -- Send to the local channel for logging
    camera.pub:send( {mp.pack(meta), c_img} )
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

while true do
  local npoll = channel_poll:poll(channel_timeout)
  local t = unix.time()
  local t_diff = t-t_debug
  if t_diff>DEBUG_INTERVAL then
    t_debug = t
    --print'debug...'
  end
end
