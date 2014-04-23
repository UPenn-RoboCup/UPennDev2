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
-- On ctrl-c
local signal = require'signal'
signal.signal("SIGINT", os.exit)
signal.signal("SIGTERM", os.exit)
-- Camera
local metadata = Config.camera[1]

-- LOGGING
local ENABLE_LOG = false
local libLog, logger, Body
if ENABLE_LOG then
	libLog = require'libLog'
	Body = require'Body'
	-- Make the logger
	logger = libLog.new('uvc',true)
end

-- Libraries
local uvc   = require'uvc'
local util  = require'util'
local torch = require'torch'

-- Extract metadata information
local w = metadata.width
local h = metadata.height
local fps = metadata.fps
local fmt = metadata.format
local name = metadata.name
local dev = metadata.dev
metadata = nil
-- Extract Config information
local operator = Config.net.operator.wired
local udp_port = Config.net.camera[name]
Config = nil

-- Debug
print(util.color('Begin','yellow'),name)

-- Open the camera
local camera = uvc.init(dev, w, h, fmt, 1, fps)

-- Torch container (Just to a pointer)
local yuyv_s = torch.ByteStorage(h*w*2,0)
local yuyv_sc = yuyv_t:cdata()
local yuyv_t = torch.ByteTensor(yuyv_s,1,torch.LongStorage{h,w})

while true do
	-- Grab the image
	local img, sz, cnt, t = camera:get_image()
  -- Set into a torch container
  local yuyv_sc.data = ffi.cast("uint8_t*",img)
	if ENABLE_LOG then
		meta.rsz = sz
		logger:record(meta, img, sz)
	end
end
