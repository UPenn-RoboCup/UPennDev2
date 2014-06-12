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
local cam_metadata = 
  Config.camera[tonumber(arg[1]) or 1]

-- Libraries
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
--[[
local ImageProc = require'ImageProc'
local ImageProc2 = require'ImageProc.ffi'
local lut_id = 
ImageProc2.load_lut (HOME.."/Data/"..cam_metadata.lut)
ImageProc2.setup(w, h)
lut_t = ImageProc2.get_lut(lut_id)
lut = lut_t:data()
--]]

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
  --t0 = unix.time()
--[[
  labelA_t = ImageProc2.yuyv_to_label(image, lut)
  cc_t = ImageProc2.color_count(labelA_t)
  labelB_t = ImageProc2.block_bitor(labelA_t)
--]]
  --t1 = unix.time()
  --print(t1-t0)
end
