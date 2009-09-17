-----------------------------------------------------------------
-- Combined Lidar manager for Team THOR
-- Reads and sends raw lidar data
-- As well as accumulate them as a map
-- and send to UDP/TCP
-- (c) Stephen McGill, Seung Joon Yi, 2013
---------------------------------
-- TODO: Critical section for include
-- Something there is non-reentrant
dofile'../include.lua'
-- Going to be threading this
local simple_ipc = require'simple_ipc'
-- For all threads
local mp = require'msgpack.MessagePack'
-- Camera
local cam_metadata = Config.camera[1]

-- Libraries
local lV    = require'libVision'
local uvc   = require'uvc'
local torch = require'torch'

-- Extract cam_metadata information
local w = cam_metadata.w
local h = cam_metadata.h
local fps = cam_metadata.fps
local fmt = cam_metadata.format
local name = cam_metadata.name
local dev = cam_metadata.dev

-- Setup the Vision system
lV.setup(w, h)
lV.load_lut(HOME.."/Data/"..cam_metadata.lut)
-- Open the camera
local camera = uvc.init(dev, w, h, fmt, 1, fps)
-- Setup the parameters
for _,v in ipairs(cam_metadata.auto_param) do
  local param, val = v[1], v[2]
  camera:set_param(param,val)
  unix.usleep(1e5)
  camera:get_param(param)
end
for _,v in ipairs(cam_metadata.param) do
  local param, val = v[1], v[2]
  camera:set_param(param,val)
  unix.usleep(1e5)
  camera:get_param(param)
end

-- Remove unneeded
cam_metadata = nil
Config = nil

-- Loop temp vars
local img, sz, cnt, t

-- On ctrl-c, saev some data
local function shutdown()
  local labelA_t, labelB_t = lV.get_labels()
  -- Save to file
  print('Saving labelA',labelA_t:size(1),labelA_t:size(2))
  local f_l = torch.DiskFile('labelA.raw', 'w')
  f_l.binary(f_l)
  f_l:writeByte(labelA_t:storage())
  f_l:close()
  --
  local jpeg = require'jpeg'
  c_yuyv = jpeg.compressor('yuyv')
  local str = c_yuyv:compress(img,w,h)
  local f_y = io.open('yuyv.jpeg','w')
  f_y:write(str)
  f_y:close()
  --
  os.exit()
end
local signal = require'signal'
signal.signal("SIGINT", shutdown)
signal.signal("SIGTERM", shutdown)

while true do
	-- Grab the image
	img, sz, cnt, t = camera:get_image()
  local t0 = unix.time()
  -- Set into a torch container
  lV.yuyv_to_labelA(img)
  lV.form_labelB()
  -- Now we can detect the ball, etc.
  local t1 = unix.time()
  print(t1-t0)
end
