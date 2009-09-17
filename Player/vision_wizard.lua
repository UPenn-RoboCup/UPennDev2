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

-- On ctrl-c
local function shutdown()
  os.exit()
end
local signal = require'signal'
signal.signal("SIGINT", shutdown)
signal.signal("SIGTERM", shutdown)

while true do
	-- Grab the image
	img, sz, cnt, t = camera:get_image()
  -- Set into a torch container
  lV.yuyv_to_labelA(img)
  lV.form_labelB()
end
