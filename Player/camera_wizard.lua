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
local udp        = require'udp'
local mp         = require'msgpack'
local unix       = require'unix'
local wait_channels = {}
local cameras = {}

-- Open each of the cameras
if Config.camera.head then
  local dev     = Config.camera.head.device
  local fps     = Config.camera.head.fps
  local quality = Config.camera.head.quality
  local width, height  = unpack(Config.camera.head.resolution)
  local camera = uvc.init(
    dev, width, height, 'yuyv', 1, fps)
  print('Opened',camera,Config.net.operator.wireless, Config.net.head_camera)
  local camera_udp_ch = udp.new_sender(
    Config.net.operator.wired, Config.net.head_camera);
  -- Create the callback functions
  local meta = {}
  meta.t = Body.get_time()
  local camera_poll = {}
  camera_poll.socket_handle = camera:descriptor()
  camera_poll.ts = Body.get_time()
  camera_poll.count = 0
  camera_poll.callback = function()
    local t_img = Body.get_time()
    local img, head_img_sz = camera:get_image()
    local t_diff = t_img - camera_poll.ts
    camera_poll.ts = t_img
    camera_poll.count = camera_poll.count + 1

    if camera_poll.count%(5*fps)==0 then
      local t_diff = t_now - camera_poll.ts
      print(string.format('Head | %5.2f FPS', camera_poll.count/t_diff))
      camera_poll.count = 0
      camera_poll.ts = t_now
    end
    camera_poll.count = camera_poll.count + 1

    -- Send over UDP
    jpeg.set_quality( quality )
    meta.t = t_img
    meta.c = 'jpeg'
    local metapack = mp.pack(meta)
    local jimg = jpeg.compress_yuyv(img,width,height)
    local udp_ret, err = camera_udp_ch:send( metapack..jimg )
    if err then print('camera udp error',err,#jimg) end
  end
  -- keep track
  table.insert(wait_channels,camera_poll)
  table.insert(cameras,camera)
end

----[[
local signal = require 'signal'
local function shutdown()
  print'Shutting down the Cameras...'
  for i,h in ipairs(cameras) do
    h:close()
    print('Closed camera',i)
  end
  os.exit()
end
signal.signal("SIGINT", shutdown)
signal.signal("SIGTERM", shutdown)
--]]

-- Poll multiple sockets
assert(#cameras>0,'No cameras available!')
local channel_poll = simple_ipc.wait_on_channels( wait_channels );
local channel_timeout = 1e3 --ms, means 1 sec timeout = 1e3
while true do
  local npoll = channel_poll:poll(channel_timeout)
end
